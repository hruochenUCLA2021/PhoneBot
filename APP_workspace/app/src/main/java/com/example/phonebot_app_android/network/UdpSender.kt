package com.example.phonebot_app_android.network

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.max

/**
 * Minimal UDP sender designed for "latest state" streaming:
 * - Producer can call [offer] at high frequency.
 * - Sender thread transmits the most recent payload (older ones may be dropped).
 */
class UdpSender {
    data class Target(val host: String, val port: Int)

    @Volatile private var running = false
    @Volatile private var target: Target = Target("192.168.20.11", 5005)

    // Cache resolved destination to avoid per-packet getByName() overhead.
    @Volatile private var cachedAddress: InetAddress? = null
    @Volatile private var cachedPort: Int = 0

    @Volatile var lastError: String? = null
        private set

    @Volatile var lastSendHz: Float? = null
        private set

    private val pendingPayload = AtomicReference<ByteArray?>(null)
    // Wake-up signal (capacity=1) to avoid polling/sleep. Still "latest-only" via pendingPayload.
    private val wake = ArrayBlockingQueue<Int>(1)
    private var thread: Thread? = null

    // Windowed-rate estimation to avoid misleading kHz spikes from tiny scheduling deltas.
    private var sentSinceReport: Int = 0
    private var lastReportNs: Long = 0L

    fun setTarget(host: String, port: Int) {
        val t = Target(host.trim(), port)
        target = t
        // Best-effort resolve now; if it fails, sender thread will report error on send attempt.
        cachedAddress = runCatching { InetAddress.getByName(t.host) }.getOrNull()
        cachedPort = t.port
    }

    fun getTarget(): Target = target

    fun start() {
        if (running) return
        running = true
        lastError = null
        lastSendHz = null

        // Ensure cache is initialized for the current target.
        val t = target
        if (cachedAddress == null || cachedPort != t.port) {
            cachedAddress = runCatching { InetAddress.getByName(t.host) }.getOrNull()
            cachedPort = t.port
        }

        thread =
            Thread(
                {
                    var socket: DatagramSocket? = null
                    try {
                        socket = DatagramSocket()
                        socket.reuseAddress = true

                        while (running) {
                            // Block until at least one payload is available (wake signal).
                            // If multiple offers happen before we wake, pendingPayload will hold the newest.
                            wake.take()

                            while (running) {
                                val payload = pendingPayload.getAndSet(null) ?: break
                                val address =
                                    cachedAddress
                                        ?: InetAddress.getByName(target.host).also { cachedAddress = it }
                                val port =
                                    cachedPort.takeIf { it in 1..65535 }
                                        ?: target.port.also { cachedPort = it }
                                val packet = DatagramPacket(payload, payload.size, address, port)
                                socket.send(packet)

                                // Update windowed average Hz (report about 4x/sec).
                                val nowNs = System.nanoTime()
                                if (lastReportNs == 0L) {
                                    lastReportNs = nowNs
                                    sentSinceReport = 0
                                }
                                sentSinceReport += 1
                                val dtNs = nowNs - lastReportNs
                                if (dtNs >= 250_000_000L) {
                                    lastSendHz = sentSinceReport.toFloat() * 1_000_000_000f / max(1L, dtNs).toFloat()
                                    sentSinceReport = 0
                                    lastReportNs = nowNs
                                }

                                // Yield CPU a bit (helps reduce load on some devices).
                                Thread.sleep(1)
                            }
                        }
                    } catch (t: Throwable) {
                        lastError = t.message ?: t::class.java.simpleName
                        Log.e("UdpSender", "UDP sender crashed", t)
                    } finally {
                        runCatching { socket?.close() }
                        running = false
                    }
                },
                "UdpSenderThread",
            ).also { it.isDaemon = true; it.start() }
    }

    fun stop() {
        running = false
        thread = null
        pendingPayload.set(null)
        wake.clear()
        lastReportNs = 0L
        sentSinceReport = 0
    }

    /** Replace any pending payload with the new one (drop older). */
    fun offer(payload: ByteArray) {
        if (!running) return
        pendingPayload.set(payload)
        // Best-effort wake; if already signaled, that's fine.
        wake.offer(1)
    }
}


