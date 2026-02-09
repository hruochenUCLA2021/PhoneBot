package com.example.phonebot_app_android.sensors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
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
    // Pre-allocate to avoid per-callback allocations/GC that can drop event rate.
    @Volatile private var latestQuat: FloatArray = FloatArray(4) // [w,x,y,z]
    @Volatile private var latestYprDeg: FloatArray = FloatArray(3) // [yaw,pitch,roll] deg
    @Volatile private var latestGameQuat: FloatArray = FloatArray(4) // [w,x,y,z]
    @Volatile private var latestGameYprDeg: FloatArray = FloatArray(3) // [yaw,pitch,roll] deg
    @Volatile private var latestAccel: FloatArray = FloatArray(3) // [x,y,z]
    @Volatile private var latestGyro: FloatArray = FloatArray(3) // [x,y,z]
    @Volatile private var latestTsNs: Long = 0L

    @Volatile private var latestRotVecHz: Float? = null
    @Volatile private var latestGameRotVecHz: Float? = null
    @Volatile private var latestAccelHz: Float? = null
    @Volatile private var latestGyroHz: Float? = null

    // UI updates should be throttled; sensors can be 100-500+ Hz.
    @Volatile private var lastUiEmitElapsedMs: Long = 0L
    // Also throttle expensive yaw/pitch/roll computation to UI rate.
    @Volatile private var lastYprComputeElapsedMs: Long = 0L

    // Temp arrays reused for Euler computation.
    private val tmpR = FloatArray(9)
    private val tmpO = FloatArray(3)

    var onUiUpdate: ((ImuState) -> Unit)? = null
    /**
     * Called without UI throttling, but ONLY when an orientation estimate updates
     * (TYPE_ROTATION_VECTOR or TYPE_GAME_ROTATION_VECTOR). Intended for UDP streaming.
     *
     * This avoids triggering network sends on high-rate accel/gyro callbacks while still
     * including the latest accel/gyro values in the snapshot payload.
     */
    var onRawUpdate: ((ImuState) -> Unit)? = null

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

    // Run sensor callbacks on a dedicated thread to avoid UI/main-thread contention dropping events.
    private var sensorThread: HandlerThread? = null
    private var sensorHandler: Handler? = null

    fun startFast() {
        val noteParts = mutableListOf<String>()
        if (rotationVector == null) {
            noteParts.add("No TYPE_ROTATION_VECTOR sensor on this device.")
        }
        if (gameRotationVector == null) {
            noteParts.add("No TYPE_GAME_ROTATION_VECTOR sensor on this device.")
        }

        // Ensure a dedicated callback thread exists.
        if (sensorThread == null) {
            sensorThread =
                HandlerThread("ImuMonitorThread").also {
                    it.start()
                    sensorHandler = Handler(it.looper)
                }
        }
        val handler = sensorHandler

        // On newer Android versions, SENSOR_DELAY_FASTEST (0us) may require
        // android.permission.HIGH_SAMPLING_RATE_SENSORS. If not granted, fall back.
        // NOTE: asking *all* sensors for 0us can be CPU heavy. User requested FASTEST for all.
        val gameFastestUs = 0 // stream trigger: fastest possible
        val auxUs = 0 // fastest possible for accel/gyro + rotvec
        val fallbackUs = 10_000 // 100 Hz as a reasonable fallback target

        try {
            // Use the overload that accepts a Handler so callbacks are NOT on the main thread.
            // Streaming trigger:
            gameRotationVector?.also { sensorManager.registerListener(this, it, gameFastestUs, 0, handler) }

            // Aux sensors (still useful, but don't need max rate for most robotics pipelines):
            rotationVector?.also { sensorManager.registerListener(this, it, auxUs, 0, handler) }
            accelerometer?.also { sensorManager.registerListener(this, it, auxUs, 0, handler) }
            gyroscope?.also { sensorManager.registerListener(this, it, auxUs, 0, handler) }

            noteParts.add("IMU: FASTEST (0us) on all sensors")
        } catch (se: SecurityException) {
            // Fall back to avoid crashing.
            sensorManager.unregisterListener(this)
            rotationVector?.also { sensorManager.registerListener(this, it, fallbackUs, 0, handler) }
            gameRotationVector?.also { sensorManager.registerListener(this, it, fallbackUs, 0, handler) }
            accelerometer?.also { sensorManager.registerListener(this, it, fallbackUs, 0, handler) }
            gyroscope?.also { sensorManager.registerListener(this, it, fallbackUs, 0, handler) }
            noteParts.add("IMU: ~100Hz (FASTEST blocked: ${se.message ?: "SecurityException"})")
        }

        onUiUpdate?.invoke(ImuState(note = noteParts.joinToString(" | ").ifBlank { null }))
    }

    fun stop() {
        sensorManager.unregisterListener(this)
        rotVecRate.reset()
        gameRotVecRate.reset()
        accelRate.reset()
        gyroRate.reset()

        sensorThread?.quitSafely()
        sensorThread = null
        sensorHandler = null
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onSensorChanged(event: SensorEvent) {
        latestTsNs = event.timestamp
        val isGameOrientationUpdate = event.sensor.type == Sensor.TYPE_GAME_ROTATION_VECTOR
        val nowMs = SystemClock.elapsedRealtime()

        when (event.sensor.type) {
            Sensor.TYPE_ROTATION_VECTOR -> {
                latestRotVecHz = rotVecRate.update(event.timestamp)
                // Quaternion (writes into our preallocated array)
                SensorManager.getQuaternionFromVector(latestQuat, event.values)

                // Euler is expensive: compute only at UI-like rate.
                if (nowMs - lastYprComputeElapsedMs >= 50) {
                    lastYprComputeElapsedMs = nowMs
                    SensorManager.getRotationMatrixFromVector(tmpR, event.values)
                    SensorManager.getOrientation(tmpR, tmpO) // radians: [azimuth(yaw), pitch, roll]
                    latestYprDeg[0] = radToDeg(tmpO[0])
                    latestYprDeg[1] = radToDeg(tmpO[1])
                    latestYprDeg[2] = radToDeg(tmpO[2])
                }
            }

            Sensor.TYPE_GAME_ROTATION_VECTOR -> {
                latestGameRotVecHz = gameRotVecRate.update(event.timestamp)
                SensorManager.getQuaternionFromVector(latestGameQuat, event.values)

                if (nowMs - lastYprComputeElapsedMs >= 50) {
                    lastYprComputeElapsedMs = nowMs
                    SensorManager.getRotationMatrixFromVector(tmpR, event.values)
                    SensorManager.getOrientation(tmpR, tmpO)
                    latestGameYprDeg[0] = radToDeg(tmpO[0])
                    latestGameYprDeg[1] = radToDeg(tmpO[1])
                    latestGameYprDeg[2] = radToDeg(tmpO[2])
                }
            }

            Sensor.TYPE_ACCELEROMETER -> {
                latestAccelHz = accelRate.update(event.timestamp)
                latestAccel[0] = event.values[0]
                latestAccel[1] = event.values[1]
                latestAccel[2] = event.values[2]
            }

            Sensor.TYPE_GYROSCOPE -> {
                latestGyroHz = gyroRate.update(event.timestamp)
                latestGyro[0] = event.values[0]
                latestGyro[1] = event.values[1]
                latestGyro[2] = event.values[2]
            }
        }

        // Fire raw update ONLY on TYPE_GAME_ROTATION_VECTOR updates.
        if (isGameOrientationUpdate) {
            onRawUpdate?.invoke(
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

        // Throttle UI updates to avoid overwhelming Compose with 100s of updates/sec.
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


