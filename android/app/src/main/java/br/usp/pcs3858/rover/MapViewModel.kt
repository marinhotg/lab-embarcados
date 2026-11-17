package br.usp.pcs3858.rover

import android.app.Application
import android.content.Context
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import br.usp.pcs3858.rover.map.OccupancyMap
import br.usp.pcs3858.rover.telemetry.FeedEvent
import br.usp.pcs3858.rover.telemetry.Simulator
import br.usp.pcs3858.rover.telemetry.Telemetry
import br.usp.pcs3858.rover.telemetry.mqttFeed
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

data class BrokerConfig(val host: String, val port: Int, val roverId: String)

sealed interface FeedState {
    data object Off : FeedState
    data class Connecting(val broker: String) : FeedState
    data class Live(val broker: String) : FeedState
    data object Simulating : FeedState
    data class Failed(val message: String) : FeedState
}

class MapViewModel(app: Application) : AndroidViewModel(app) {

    private val prefs = app.getSharedPreferences("map", Context.MODE_PRIVATE)
    private var feed: Job? = null

    val map = OccupancyMap()

    private val _config = MutableStateFlow(
        BrokerConfig(
            host = prefs.getString("host", ROVER_BROKER) ?: ROVER_BROKER,
            port = prefs.getInt("port", 1883),
            roverId = prefs.getString("rover_id", "01") ?: "01",
        )
    )
    val config: StateFlow<BrokerConfig> = _config.asStateFlow()

    private val _state = MutableStateFlow<FeedState>(FeedState.Off)
    val state: StateFlow<FeedState> = _state.asStateFlow()

    private val _latest = MutableStateFlow<Telemetry?>(null)
    val latest: StateFlow<Telemetry?> = _latest.asStateFlow()

    /* Bumped on every map change so the canvas redraws. */
    private val _version = MutableStateFlow(0)
    val version: StateFlow<Int> = _version.asStateFlow()

    fun connect(config: BrokerConfig) {
        _config.value = config
        prefs.edit()
            .putString("host", config.host)
            .putInt("port", config.port)
            .putString("rover_id", config.roverId)
            .apply()

        val broker = "${config.host}:${config.port}"
        restart {
            _state.value = FeedState.Connecting(broker)
            mqttFeed(config.host, config.port, config.roverId).collect { event ->
                when (event) {
                    FeedEvent.Connected -> _state.value = FeedState.Live(broker)
                    is FeedEvent.Lost -> _state.value = FeedState.Failed("Broker $broker: ${event.reason}")
                    is FeedEvent.Message -> Telemetry.parse(event.payload)?.let(::ingest)
                }
            }
        }
    }

    fun simulate() = restart {
        _state.value = FeedState.Simulating
        Simulator.feed().collect(::ingest)
    }

    fun stop() {
        feed?.cancel()
        feed = null
        _state.value = FeedState.Off
    }

    fun clearMap() {
        map.reset()
        _latest.value = null
        _version.value++
    }

    private fun restart(block: suspend () -> Unit) {
        feed?.cancel()
        clearMap()
        feed = viewModelScope.launch { block() }
    }

    private fun ingest(t: Telemetry) {
        map.ingest(t)
        _latest.value = t
        _version.value++
    }

    private companion object {
        /* The broker the rover hosts on its own access point. */
        const val ROVER_BROKER = "192.168.4.1"
    }
}
