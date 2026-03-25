package com.example.phonebot_app_android.llm

import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import java.io.File
import java.io.RandomAccessFile
import java.util.concurrent.atomic.AtomicBoolean

class WavAudioRecorder(
    private val sampleRateHz: Int = 16000,
    private val channelConfig: Int = AudioFormat.CHANNEL_IN_MONO,
    private val audioFormat: Int = AudioFormat.ENCODING_PCM_16BIT,
) {
    private val running = AtomicBoolean(false)
    private var thread: Thread? = null
    private var record: AudioRecord? = null

    fun isRunning(): Boolean = running.get()

    fun start(outputWav: File) {
        if (!running.compareAndSet(false, true)) return

        val minBuf = AudioRecord.getMinBufferSize(sampleRateHz, channelConfig, audioFormat)
        val bufSize = (minBuf * 2).coerceAtLeast(sampleRateHz) // some slack

        val ar =
            AudioRecord(
                MediaRecorder.AudioSource.MIC,
                sampleRateHz,
                channelConfig,
                audioFormat,
                bufSize,
            )
        record = ar

        // Create file and write placeholder WAV header.
        outputWav.parentFile?.mkdirs()
        val raf = RandomAccessFile(outputWav, "rw")
        raf.setLength(0)
        writeWavHeader(
            raf = raf,
            sampleRateHz = sampleRateHz,
            channels = 1,
            bitsPerSample = 16,
            dataLenBytes = 0,
        )

        val t =
            Thread {
                val buf = ByteArray(bufSize)
                var dataLen: Long = 0L
                try {
                    ar.startRecording()
                    while (running.get()) {
                        val n = ar.read(buf, 0, buf.size)
                        if (n > 0) {
                            raf.write(buf, 0, n)
                            dataLen += n.toLong()
                        }
                    }
                } finally {
                    runCatching { ar.stop() }
                    runCatching { ar.release() }
                    record = null
                    // Patch header with final data length.
                    runCatching {
                        raf.seek(0)
                        writeWavHeader(
                            raf = raf,
                            sampleRateHz = sampleRateHz,
                            channels = 1,
                            bitsPerSample = 16,
                            dataLenBytes = dataLen,
                        )
                    }
                    runCatching { raf.close() }
                    running.set(false)
                }
            }.also { it.name = "WavAudioRecorder" }
        thread = t
        t.start()
    }

    fun stop() {
        running.set(false)
        thread?.join(1500)
        thread = null
    }

    private fun writeWavHeader(
        raf: RandomAccessFile,
        sampleRateHz: Int,
        channels: Int,
        bitsPerSample: Int,
        dataLenBytes: Long,
    ) {
        val byteRate = sampleRateHz * channels * bitsPerSample / 8
        val blockAlign = (channels * bitsPerSample / 8)
        val riffChunkSize = 36 + dataLenBytes

        raf.writeBytes("RIFF")
        raf.writeIntLE(riffChunkSize.toInt())
        raf.writeBytes("WAVE")
        raf.writeBytes("fmt ")
        raf.writeIntLE(16) // PCM fmt chunk size
        raf.writeShortLE(1) // audio format PCM
        raf.writeShortLE(channels.toShort())
        raf.writeIntLE(sampleRateHz)
        raf.writeIntLE(byteRate)
        raf.writeShortLE(blockAlign.toShort())
        raf.writeShortLE(bitsPerSample.toShort())
        raf.writeBytes("data")
        raf.writeIntLE(dataLenBytes.toInt())
    }
}

private fun RandomAccessFile.writeIntLE(v: Int) {
    write(byteArrayOf((v and 0xFF).toByte(), ((v shr 8) and 0xFF).toByte(), ((v shr 16) and 0xFF).toByte(), ((v shr 24) and 0xFF).toByte()))
}

private fun RandomAccessFile.writeShortLE(v: Short) {
    val i = v.toInt()
    write(byteArrayOf((i and 0xFF).toByte(), ((i shr 8) and 0xFF).toByte()))
}

