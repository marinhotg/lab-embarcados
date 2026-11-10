package br.usp.pcs3858.rover.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Slider
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import br.usp.pcs3858.rover.Command
import br.usp.pcs3858.rover.LinkState
import br.usp.pcs3858.rover.PairedDevice
import br.usp.pcs3858.rover.RoverViewModel

private val StopRed = Color(0xFFD6342C)

@Composable
fun ControlScreen(
    vm: RoverViewModel,
    hasPermission: Boolean,
    onRequestPermission: () -> Unit,
) {
    val state by vm.state.collectAsStateWithLifecycle()
    val devices by vm.devices.collectAsStateWithLifecycle()
    val command by vm.command.collectAsStateWithLifecycle()
    val limit by vm.limit.collectAsStateWithLifecycle()

    LaunchedEffect(hasPermission) {
        if (hasPermission) vm.refreshDevices()
    }

    Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
        Column(
            modifier = Modifier
                .safeDrawingPadding()
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            when {
                !vm.bluetoothAvailable ->
                    Text("Este aparelho não tem Bluetooth.")
                !hasPermission -> {
                    Text("O app precisa da permissão de Bluetooth para falar com o rover.")
                    Button(onClick = onRequestPermission) { Text("Permitir Bluetooth") }
                }
                else -> ConnectionBar(
                    state = state,
                    devices = devices,
                    onRefresh = vm::refreshDevices,
                    onConnect = vm::connect,
                    onDisconnect = vm::disconnect,
                )
            }

            val connected = state is LinkState.Connected
            Spacer(Modifier.weight(1f))
            Readout(command)
            Joystick(
                onChange = vm::onJoystick,
                ring = if (connected) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.outline,
                knob = if (connected) MaterialTheme.colorScheme.primary else MaterialTheme.colorScheme.outlineVariant,
            )
            Column(modifier = Modifier.fillMaxWidth()) {
                Text("Limite: $limit %", style = MaterialTheme.typography.labelLarge)
                Slider(
                    value = limit.toFloat(),
                    onValueChange = { vm.setLimit(it.toInt()) },
                    valueRange = 10f..100f,
                    steps = 8,
                )
            }
            Spacer(Modifier.weight(1f))
            Button(
                onClick = vm::stop,
                enabled = connected,
                colors = ButtonDefaults.buttonColors(containerColor = StopRed, contentColor = Color.White),
                modifier = Modifier
                    .fillMaxWidth()
                    .height(72.dp),
            ) {
                Text("PARAR", fontSize = 24.sp, fontWeight = FontWeight.Bold)
            }
        }
    }
}

@Composable
private fun ConnectionBar(
    state: LinkState,
    devices: List<PairedDevice>,
    onRefresh: () -> Unit,
    onConnect: (PairedDevice) -> Unit,
    onDisconnect: () -> Unit,
) {
    var menuOpen by remember { mutableStateOf(false) }
    var selected by remember(devices) { mutableStateOf(devices.firstOrNull()) }

    Column(modifier = Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text(
            text = when (state) {
                LinkState.Idle -> "Desconectado"
                is LinkState.Connecting -> "Conectando em ${state.name}…"
                is LinkState.Connected -> "Conectado em ${state.name}"
                is LinkState.Failed -> state.message
            },
            style = MaterialTheme.typography.titleMedium,
            color = when (state) {
                is LinkState.Connected -> MaterialTheme.colorScheme.primary
                is LinkState.Failed -> MaterialTheme.colorScheme.error
                else -> MaterialTheme.colorScheme.onBackground
            },
        )

        if (state is LinkState.Connected) {
            OutlinedButton(onClick = onDisconnect, modifier = Modifier.fillMaxWidth()) { Text("Desconectar") }
            return@Column
        }

        if (devices.isEmpty()) {
            Text("Nenhum aparelho pareado. Pareie o \"Rover\" nas configurações de Bluetooth do Android.")
            OutlinedButton(onClick = onRefresh) { Text("Atualizar lista") }
            return@Column
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
            Column(modifier = Modifier.weight(1f)) {
                OutlinedButton(onClick = { menuOpen = true }, modifier = Modifier.fillMaxWidth()) {
                    Text(selected?.name ?: "Escolher aparelho")
                }
                DropdownMenu(expanded = menuOpen, onDismissRequest = { menuOpen = false }) {
                    devices.forEach { d ->
                        DropdownMenuItem(
                            text = { Text("${d.name}  ${d.address}") },
                            onClick = { selected = d; menuOpen = false },
                        )
                    }
                }
            }
            Button(
                onClick = { selected?.let(onConnect) },
                enabled = selected != null && state !is LinkState.Connecting,
            ) { Text("Conectar") }
        }
    }
}

@Composable
private fun Readout(command: Command) {
    Row(horizontalArrangement = Arrangement.spacedBy(24.dp)) {
        Text("Linear %+4d %%".format(command.linear), fontFamily = FontFamily.Monospace)
        Text("Rotação %+4d %%".format(command.rotation), fontFamily = FontFamily.Monospace)
    }
}
