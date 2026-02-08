package com.example.phonebot_app_android.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import kotlin.math.PI

data class ImuState(
    val timestampNs: Long = 0L,
    val quat: FloatArray? = null, // [w, x, y, z]
    val yprDeg: FloatArray? = null, // [yaw, pitch, roll] degrees
    val accel: FloatArray? = null, // [x,y,z] m/s^2
    val gyro: FloatArray? = null, // [x,y,z] rad/s
    val note: String? = null,
)

class ImuMonitor(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    private val rotationVector: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val accelerometer: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    // latest samples (updated at SENSOR_DELAY_FASTEST)
    @Volatile private var latestQuat: FloatArray? = null
    @Volatile private var latestYprDeg: FloatArray? = null
    @Volatile private var latestAccel: FloatArray? = null
    @Volatile private var latestGyro: FloatArray? = null
    @Volatile private var latestTsNs: Long = 0L

    // UI updates should be throttled; sensors can be 100-500+ Hz.
    @Volatile private var lastUiEmitElapsedMs: Long = 0L

    var onUiUpdate: ((ImuState) -> Unit)? = null

    fun startFast() {
        val noteParts = mutableListOf<String>()
        if (rotationVector == null) {
            noteParts.add("No TYPE_ROTATION_VECTOR sensor on this device.")
        }

        // On newer Android versions, SENSOR_DELAY_FASTEST (0us) may require
        // android.permission.HIGH_SAMPLING_RATE_SENSORS. If not granted, fall back.
        val fastestDelay = SensorManager.SENSOR_DELAY_FASTEST
        val fallbackDelay = SensorManager.SENSOR_DELAY_GAME

        try {
            rotationVector?.also { sensorManager.registerListener(this, it, fastestDelay) }
            accelerometer?.also { sensorManager.registerListener(this, it, fastestDelay) }
            gyroscope?.also { sensorManager.registerListener(this, it, fastestDelay) }
            noteParts.add("IMU rate: FASTEST")
        } catch (se: SecurityException) {
            // Fall back to avoid crashing.
            sensorManager.unregisterListener(this)
            rotationVector?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            accelerometer?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            gyroscope?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            noteParts.add("IMU rate: GAME (FASTEST blocked: ${se.message ?: "SecurityException"})")
        }

        onUiUpdate?.invoke(ImuState(note = noteParts.joinToString(" | ").ifBlank { null }))
    }

    fun stop() {
        sensorManager.unregisterListener(this)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onSensorChanged(event: SensorEvent) {
        latestTsNs = event.timestamp

        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> {
                // Quaternion
                val q = FloatArray(4)
                SensorManager.getQuaternionFromVector(q, event.values)
                latestQuat = q // [w,x,y,z]

                // Euler (yaw/pitch/roll) from rotation matrix
                val r = FloatArray(9)
                val o = FloatArray(3)
                SensorManager.getRotationMatrixFromVector(r, event.values)
                SensorManager.getOrientation(r, o) // radians: [azimuth(yaw), pitch, roll]
                latestYprDeg =
                    floatArrayOf(
                        radToDeg(o[0]),
                        radToDeg(o[1]),
                        radToDeg(o[2]),
                    )
            }

            Sensor.TYPE_ACCELEROMETER -> {
                latestAccel = event.values.clone()
            }

            Sensor.TYPE_GYROSCOPE -> {
                latestGyro = event.values.clone()
            }
        }

        // Throttle UI updates to avoid overwhelming Compose with 100s of updates/sec.
        val nowMs = SystemClock.elapsedRealtime()
        if (nowMs - lastUiEmitElapsedMs < 50) return // ~20 Hz UI updates
        lastUiEmitElapsedMs = nowMs

        onUiUpdate?.invoke(
            ImuState(
                timestampNs = latestTsNs,
                quat = latestQuat,
                yprDeg = latestYprDeg,
                accel = latestAccel,
                gyro = latestGyro,
            )
        )
    }

    private fun radToDeg(rad: Float): Float = (rad * (180.0f / PI.toFloat()))
}


