package com.example.phonebot_app_android.policy

import android.content.Context
import org.json.JSONObject
import org.tensorflow.lite.Interpreter
import java.io.Closeable
import java.io.FileInputStream
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel

data class PolicyMeta(
    val envName: String,
    val stateDim: Int,
    val actionDim: Int,
    val defaultPose: FloatArray,
)

data class PolicyAsset(
    val tfliteFile: String, // file name under assets/exported_tflite/
    val metaFile: String,   // file name under assets/exported_tflite/
    val meta: PolicyMeta,
) {
    val mode: String = if (meta.envName.contains("Alter", ignoreCase = true)) "alter" else "normal"
}

object PhonebotPolicyAssets {
    const val ASSET_DIR = "exported_tflite"

    /**
     * Default policy when this `.tflite` (and matching `_metadata.json`) exists in assets.
     * Otherwise the app falls back to the first entry in [listAvailablePolicies] (alphabetical `.tflite` order).
     */
    const val PREFERRED_DEFAULT_TFLITE = "phonebot_flat_alter_fv2_torque_awared_home_straight_v1_actor.tflite"

    fun listAvailablePolicies(context: Context): List<PolicyAsset> {
        val files = context.assets.list(ASSET_DIR)?.toList().orEmpty()
        val tflites = files.filter { it.endsWith(".tflite", ignoreCase = true) }.sorted()
        val out = ArrayList<PolicyAsset>()
        for (t in tflites) {
            val metaGuess = t.removeSuffix(".tflite") + "_metadata.json"
            if (metaGuess !in files) continue
            val meta = readMeta(context, metaGuess) ?: continue
            out.add(PolicyAsset(tfliteFile = t, metaFile = metaGuess, meta = meta))
        }
        return out
    }

    /** Pick initial dropdown selection: preferred file if listed, else first sorted policy. */
    fun defaultSelectedFilename(options: List<PolicyAsset>): String {
        if (options.isEmpty()) return ""
        return options.firstOrNull { it.tfliteFile == PREFERRED_DEFAULT_TFLITE }?.tfliteFile
            ?: options.first().tfliteFile
    }

    fun readMeta(context: Context, metaFile: String): PolicyMeta? {
        return try {
            val s = context.assets.open("$ASSET_DIR/$metaFile").use { it.readBytes().decodeToString() }
            val j = JSONObject(s)
            val env = j.optString("env_name", "")
            val stateDim = j.optInt("state_dim", 52)
            val actionDim = j.optInt("action_dim", 13)
            val arr = j.optJSONArray("default_pose")
            val dp = FloatArray(13)
            if (arr != null) {
                val n = minOf(dp.size, arr.length())
                for (i in 0 until n) dp[i] = arr.optDouble(i, 0.0).toFloat()
            }
            PolicyMeta(envName = env, stateDim = stateDim, actionDim = actionDim, defaultPose = dp)
        } catch (_: Throwable) {
            null
        }
    }
}

class TfliteActor private constructor(
    private val interpreter: Interpreter,
    val stateDim: Int,
    val actionDim: Int,
) : Closeable {
    fun run(state: FloatArray): FloatArray {
        require(state.size == stateDim) { "state size ${state.size} != expected $stateDim" }
        val input = arrayOf(state)
        val out = Array(1) { FloatArray(actionDim) }
        interpreter.run(input, out)
        return out[0]
    }

    override fun close() {
        runCatching { interpreter.close() }
    }

    companion object {
        fun fromAsset(context: Context, tfliteFile: String, stateDim: Int, actionDim: Int): TfliteActor {
            val mbb = loadModelFile(context, "${PhonebotPolicyAssets.ASSET_DIR}/$tfliteFile")
            val interpreter = Interpreter(mbb, Interpreter.Options().apply { setNumThreads(2) })
            return TfliteActor(interpreter = interpreter, stateDim = stateDim, actionDim = actionDim)
        }

        private fun loadModelFile(context: Context, assetPath: String): MappedByteBuffer {
            val afd = context.assets.openFd(assetPath)
            FileInputStream(afd.fileDescriptor).channel.use { ch ->
                return ch.map(FileChannel.MapMode.READ_ONLY, afd.startOffset, afd.declaredLength)
            }
        }
    }
}

