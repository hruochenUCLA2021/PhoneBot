package com.example.phonebot_app_android.network

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Minimal UDP receiver (single thread) to listen for PC -> Android motor command packets.
 */
class UdpReceiver {
    private val running = AtomicBoolean(false)
    private var thread: Thread? = null
    private var socket: DatagramSocket? = null

    @Volatile var lastError: String? = null
        private set

    private class RateEstimator(private val alpha: Float = 0.1f) {
        private var lastNs: Long = 0L
        private var emaHz: Float? = null

        fun update(nowNs: Long): Float? {
            if (lastNs != 0L) {
                val dt = nowNs - lastNs
                if (dt > 0) {
                    val instHz = 1_000_000_000f / dt.toFloat()
                    emaHz = if (emaHz == null) instHz else (emaHz!! * (1 - alpha) + instHz * alpha)
                }
            }
            lastNs = nowNs
            return emaHz
        }
    }

    private val rate = RateEstimator()

    var onMotorPacket: ((PhonebotProtocol.MotorPacket, Float?) -> Unit)? = null

    fun start(listenPort: Int) {
        stop()
        running.set(true)
        lastError = null

        thread =
            Thread(
                {
                    val buf = ByteArray(2048)
                    var localSocket: DatagramSocket? = null
                    try {
                        localSocket = DatagramSocket(null).apply {
                            reuseAddress = true
                            bind(InetSocketAddress(listenPort))
                        }
                        socket = localSocket
                        val packet = DatagramPacket(buf, buf.size)
                        while (running.get()) {
                            localSocket.receive(packet)
                            val payload = packet.data.copyOfRange(0, packet.length)
                            val motor = PhonebotProtocol.tryParseMotorPacket(payload) ?: continue
                            val hz = rate.update(System.nanoTime())
                            onMotorPacket?.invoke(motor, hz)
                        }
                    } catch (t: Throwable) {
                        if (running.get()) {
                            lastError = t.message ?: t::class.java.simpleName
                            Log.e("UdpReceiver", "UDP receiver crashed", t)
                        }
                    } finally {
                        runCatching { localSocket?.close() }
                        socket = null
                        running.set(false)
                    }
                },
                "UdpReceiverThread",
            ).also { it.isDaemon = true; it.start() }
    }

    fun stop() {
        running.set(false)
        runCatching { socket?.close() }
        socket = null
        thread = null
    }
}


