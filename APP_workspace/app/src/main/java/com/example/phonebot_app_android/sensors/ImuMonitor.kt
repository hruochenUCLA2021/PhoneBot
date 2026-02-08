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
    val gameQuat: FloatArray? = null, // [w, x, y, z] from TYPE_GAME_ROTATION_VECTOR
    val gameYprDeg: FloatArray? = null, // [yaw, pitch, roll] degrees from TYPE_GAME_ROTATION_VECTOR
    val accel: FloatArray? = null, // [x,y,z] m/s^2
    val gyro: FloatArray? = null, // [x,y,z] rad/s
    // Estimated delivered sensor rates (Hz). These measure callbacks frequency, not UI frequency.
    val rotVecHz: Float? = null,
    val gameRotVecHz: Float? = null,
    val accelHz: Float? = null,
    val gyroHz: Float? = null,
    val note: String? = null,
)

class ImuMonitor(context: Context) : SensorEventListener {
    private val sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

    private val rotationVector: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val gameRotationVector: Sensor? =
        sensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
    private val accelerometer: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyroscope: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    // latest samples (updated at SENSOR_DELAY_FASTEST)
    @Volatile private var latestQuat: FloatArray? = null
    @Volatile private var latestYprDeg: FloatArray? = null
    @Volatile private var latestGameQuat: FloatArray? = null
    @Volatile private var latestGameYprDeg: FloatArray? = null
    @Volatile private var latestAccel: FloatArray? = null
    @Volatile private var latestGyro: FloatArray? = null
    @Volatile private var latestTsNs: Long = 0L

    @Volatile private var latestRotVecHz: Float? = null
    @Volatile private var latestGameRotVecHz: Float? = null
    @Volatile private var latestAccelHz: Float? = null
    @Volatile private var latestGyroHz: Float? = null

    // UI updates should be throttled; sensors can be 100-500+ Hz.
    @Volatile private var lastUiEmitElapsedMs: Long = 0L

    var onUiUpdate: ((ImuState) -> Unit)? = null

    private class RateEstimator(private val alpha: Float = 0.1f) {
        private var lastTsNs: Long = 0L
        private var emaHz: Float? = null

        fun update(tsNs: Long): Float? {
            if (lastTsNs != 0L) {
                val dt = tsNs - lastTsNs
                if (dt > 0) {
                    val instHz = 1_000_000_000f / dt.toFloat()
                    emaHz = if (emaHz == null) instHz else (emaHz!! * (1 - alpha) + instHz * alpha)
                }
            }
            lastTsNs = tsNs
            return emaHz
        }

        fun reset() {
            lastTsNs = 0L
            emaHz = null
        }
    }

    private val rotVecRate = RateEstimator()
    private val gameRotVecRate = RateEstimator()
    private val accelRate = RateEstimator()
    private val gyroRate = RateEstimator()

    fun startFast() {
        val noteParts = mutableListOf<String>()
        if (rotationVector == null) {
            noteParts.add("No TYPE_ROTATION_VECTOR sensor on this device.")
        }
        if (gameRotationVector == null) {
            noteParts.add("No TYPE_GAME_ROTATION_VECTOR sensor on this device.")
        }

        // On newer Android versions, SENSOR_DELAY_FASTEST (0us) may require
        // android.permission.HIGH_SAMPLING_RATE_SENSORS. If not granted, fall back.
        val fastestDelay = SensorManager.SENSOR_DELAY_FASTEST
        val fallbackDelay = SensorManager.SENSOR_DELAY_GAME

        try {
            rotationVector?.also { sensorManager.registerListener(this, it, fastestDelay) }
            gameRotationVector?.also { sensorManager.registerListener(this, it, fastestDelay) }
            accelerometer?.also { sensorManager.registerListener(this, it, fastestDelay) }
            gyroscope?.also { sensorManager.registerListener(this, it, fastestDelay) }
            noteParts.add("IMU rate: FASTEST")
        } catch (se: SecurityException) {
            // Fall back to avoid crashing.
            sensorManager.unregisterListener(this)
            rotationVector?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            gameRotationVector?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            accelerometer?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            gyroscope?.also { sensorManager.registerListener(this, it, fallbackDelay) }
            noteParts.add("IMU rate: GAME (FASTEST blocked: ${se.message ?: "SecurityException"})")
        }

        onUiUpdate?.invoke(ImuState(note = noteParts.joinToString(" | ").ifBlank { null }))
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        rotVecRate.reset()
        gameRotVecRate.reset()
        accelRate.reset()
        gyroRate.reset()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onSensorChanged(event: SensorEvent) {
        latestTsNs = event.timestamp

        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> {
                latestRotVecHz = rotVecRate.update(event.timestamp)
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

            Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                latestGameRotVecHz = gameRotVecRate.update(event.timestamp)
                val q = FloatArray(4)
                SensorManager.getQuaternionFromVector(q, event.values)
                latestGameQuat = q

                val r = FloatArray(9)
                val o = FloatArray(3)
                SensorManager.getRotationMatrixFromVector(r, event.values)
                SensorManager.getOrientation(r, o)
                latestGameYprDeg =
                    floatArrayOf(
                        radToDeg(o[0]),
                        radToDeg(o[1]),
                        radToDeg(o[2]),
                    )
            }

            Sensor.TYPE_ACCELEROMETER -> {
                latestAccelHz = accelRate.update(event.timestamp)
                latestAccel = event.values.clone()
            }

            Sensor.TYPE_GYROSCOPE -> {
                latestGyroHz = gyroRate.update(event.timestamp)
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
                gameQuat = latestGameQuat,
                gameYprDeg = latestGameYprDeg,
                accel = latestAccel,
                gyro = latestGyro,
                rotVecHz = latestRotVecHz,
                gameRotVecHz = latestGameRotVecHz,
                accelHz = latestAccelHz,
                gyroHz = latestGyroHz,
            )
        )
    }

    private fun radToDeg(rad: Float): Float = (rad * (180.0f / PI.toFloat()))
}


