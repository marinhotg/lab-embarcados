package br.usp.pcs3858.rover

import android.annotation.SuppressLint
import android.app.Application
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import br.usp.pcs3858.rover.link.RoverLink
import java.io.IOException
import kotlin.math.roundToInt
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

sealed interface LinkState {
    data object Idle : LinkState
    data class Connecting(val name: String) : LinkState
    data class Connected(val name: String) : LinkState
    data class Failed(val message: String) : LinkState
}

data class PairedDevice(val name: String, val address: String)

data class Command(val linear: Int = 0, val rotation: Int = 0)

class RoverViewModel(app: Application) : AndroidViewModel(app) {

    private val adapter =
        app.getSystemService(BluetoothManager::class.java)?.adapter
    private val link = RoverLink()
    private var sender: Job? = null

    private val _state = MutableStateFlow<LinkState>(LinkState.Idle)
    val state: StateFlow<LinkState> = _state.asStateFlow()

    private val _devices = MutableStateFlow<List<PairedDevice>>(emptyList())
    val devices: StateFlow<List<PairedDevice>> = _devices.asStateFlow()

    private val _command = MutableStateFlow(Command())
    val command: StateFlow<Command> = _command.asStateFlow()

    private val _limit = MutableStateFlow(60)
    val limit: StateFlow<Int> = _limit.asStateFlow()

    val bluetoothAvailable: Boolean get() = adapter != null

    @SuppressLint("MissingPermission")
    fun refreshDevices() {
        val bonded = runCatching { adapter?.bondedDevices.orEmpty() }.getOrDefault(emptySet())
        _devices.value = bonded
            .map { PairedDevice(it.name ?: it.address, it.address) }
            .sortedWith(compareByDescending<PairedDevice> { it.name == ROVER_NAME }.thenBy { it.name })
    }

    fun connect(device: PairedDevice) {
        val remote: BluetoothDevice = adapter?.getRemoteDevice(device.address) ?: return
        sender?.cancel()
        _command.value = Command()
        _state.value = LinkState.Connecting(device.name)

        viewModelScope.launch {
            try {
                link.connect(remote)
            } catch (e: IOException) {
                _state.value = LinkState.Failed("Não conectou em ${device.name}. Está ligado e pareado?")
                return@launch
            } catch (e: SecurityException) {
                _state.value = LinkState.Failed("Sem permissão de Bluetooth.")
                return@launch
            }
            _state.value = LinkState.Connected(device.name)
            startSender()
        }
    }

    fun disconnect() {
        sender?.cancel()
        sender = null
        _command.value = Command()
        viewModelScope.launch {
            runCatching { link.stop() }
            link.close()
            _state.value = LinkState.Idle
        }
    }

    /* Positive rotation is counter-clockwise, so stick right is negative. */
    fun onJoystick(x: Float, y: Float) {
        val scale = _limit.value
        _command.value = Command(
            linear = (y * scale).roundToInt(),
            rotation = (-x * scale).roundToInt(),
        )
    }

    fun setLimit(percent: Int) {
        _limit.value = percent.coerceIn(10, 100)
    }

    /* Sent right away, not on the next period. */
    fun stop() {
        _command.value = Command()
        viewModelScope.launch { runCatching { link.stop() } }
    }

    private fun startSender() {
        sender?.cancel()
        sender = viewModelScope.launch {
            while (isActive) {
                val c = _command.value
                try {
                    link.send(c.linear, c.rotation)
                } catch (e: IOException) {
                    link.close()
                    _command.value = Command()
                    _state.value = LinkState.Failed("Conexão perdida. O rover parou sozinho pelo timeout.")
                    return@launch
                }
                delay(PERIOD_MS)
            }
        }
    }

    override fun onCleared() {
        sender?.cancel()
    }

    private companion object {
        const val ROVER_NAME = "Rover"
        const val PERIOD_MS = 100L
    }
}
