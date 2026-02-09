package com.example.phonebot_app_android.network

import android.util.Log
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.atomic.AtomicReference

/**
 * Minimal UDP sender designed for "latest state" streaming:
 * - Producer can call [offer] at high frequency.
 * - Sender thread transmits the most recent payload (older ones may be dropped).
 */
class UdpSender {
    data class Target(val host: String, val port: Int)

    @Volatile private var running = false
    @Volatile private var target: Target = Target("192.168.1.2", 5005)

    @Volatile var lastError: String? = null
        private set

    private val pendingPayload = AtomicReference<ByteArray?>(null)
    private var thread: Thread? = null

    fun setTarget(host: String, port: Int) {
        target = Target(host.trim(), port)
    }

    fun getTarget(): Target = target

    fun start() {
        if (running) return
        running = true
        lastError = null

        thread =
            Thread(
                {
                    var socket: DatagramSocket? = null
                    try {
                        socket = DatagramSocket()
                        socket.reuseAddress = true

                        while (running) {
                            val payload = pendingPayload.getAndSet(null)
                            if (payload == null) {
                                // Avoid busy spinning.
                                Thread.sleep(1)
                                continue
                            }

                            val t = target
                            val address = InetAddress.getByName(t.host)
                            val packet = DatagramPacket(payload, payload.size, address, t.port)
                            socket.send(packet)
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
    }

    /** Replace any pending payload with the new one (drop older). */
    fun offer(payload: ByteArray) {
        if (!running) return
        pendingPayload.set(payload)
    }
}


