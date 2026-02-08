package com.example.phonebot_app_android.system

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import androidx.core.content.ContextCompat
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

data class BatteryState(
    val percent: Int? = null,
    val isCharging: Boolean? = null,
    val plugged: String? = null,
    val status: String? = null,
)

class BatteryMonitor(private val context: Context) {
    private val _state = MutableStateFlow(BatteryState())
    val state: StateFlow<BatteryState> = _state

    private val receiver =
        object : BroadcastReceiver() {
            override fun onReceive(ctx: Context?, intent: Intent?) {
                if (intent == null) return

                val level = intent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1)
                val scale = intent.getIntExtra(BatteryManager.EXTRA_SCALE, -1)
                val percent =
                    if (level >= 0 && scale > 0) ((level * 100f) / scale).toInt() else null

                val statusInt = intent.getIntExtra(BatteryManager.EXTRA_STATUS, -1)
                val isCharging =
                    statusInt == BatteryManager.BATTERY_STATUS_CHARGING ||
                        statusInt == BatteryManager.BATTERY_STATUS_FULL

                val pluggedInt = intent.getIntExtra(BatteryManager.EXTRA_PLUGGED, 0)
                val plugged =
                    when (pluggedInt) {
                        BatteryManager.BATTERY_PLUGGED_AC -> "AC"
                        BatteryManager.BATTERY_PLUGGED_USB -> "USB"
                        BatteryManager.BATTERY_PLUGGED_WIRELESS -> "WIRELESS"
                        else -> "UNPLUGGED"
                    }

                val status =
                    when (statusInt) {
                        BatteryManager.BATTERY_STATUS_CHARGING -> "CHARGING"
                        BatteryManager.BATTERY_STATUS_DISCHARGING -> "DISCHARGING"
                        BatteryManager.BATTERY_STATUS_FULL -> "FULL"
                        BatteryManager.BATTERY_STATUS_NOT_CHARGING -> "NOT_CHARGING"
                        else -> "UNKNOWN"
                    }

                _state.value =
                    BatteryState(
                        percent = percent,
                        isCharging = isCharging,
                        plugged = plugged,
                        status = status,
                    )
            }
        }

    fun start() {
        // Get an immediate sticky update if available.
        //
        // Use ContextCompat.registerReceiver to satisfy newer Android behavior changes around
        // dynamic receiver export flags.
        ContextCompat.registerReceiver(
            context,
            receiver,
            IntentFilter(Intent.ACTION_BATTERY_CHANGED),
            ContextCompat.RECEIVER_NOT_EXPORTED,
        )
    }

    fun stop() {
        runCatching { context.unregisterReceiver(receiver) }
    }
}


