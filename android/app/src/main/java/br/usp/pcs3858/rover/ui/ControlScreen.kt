package br.usp.pcs3858.rover.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.safeDrawingPadding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import br.usp.pcs3858.rover.BrokerConfig
import br.usp.pcs3858.rover.Command
import br.usp.pcs3858.rover.FeedState
import br.usp.pcs3858.rover.LinkState
import br.usp.pcs3858.rover.MapViewModel
import br.usp.pcs3858.rover.PairedDevice
import br.usp.pcs3858.rover.RoverViewModel
import br.usp.pcs3858.rover.telemetry.Telemetry

private val StopRed = Color(0xFFD6342C)

@Composable
fun ControlScreen(
    vm: RoverViewModel,
    mapVm: MapViewModel,
    hasPermission: Boolean,
    onRequestPermission: () -> Unit,
) {
    val state by vm.state.collectAsStateWithLifecycle()
    val devices by vm.devices.collectAsStateWithLifecycle()
    val command by vm.command.collectAsStateWithLifecycle()
    val limit by vm.limit.collectAsStateWithLifecycle()
    val feed by mapVm.state.collectAsStateWithLifecycle()
    val latest by mapVm.latest.collectAsStateWithLifecycle()
    val version by mapVm.version.collectAsStateWithLifecycle()
    val config by mapVm.config.collectAsStateWithLifecycle()
    var settingsOpen by remember { mutableStateOf(false) }

    LaunchedEffect(hasPermission) {
        if (hasPermission) vm.refreshDevices()
    }

    Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
        Column(
            modifier = Modifier
                .safeDrawingPadding()
                .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
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

            FeedBar(feed = feed, onOpen = { settingsOpen = true })

            val colors = MaterialTheme.colorScheme
            MapView(
                map = mapVm.map,
                version = version,
                unknown = colors.surfaceVariant,
                free = colors.surface,
                occupied = colors.onSurface,
                track = colors.primary,
                rover = colors.tertiary,
                modifier = Modifier
                    .fillMaxWidth()
                    .weight(1f)
                    .clip(RoundedCornerShape(12.dp)),
            )

            SonarRow(latest)

            val connected = state is LinkState.Connected
            Row(
                horizontalArrangement = Arrangement.spacedBy(12.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Joystick(
                    onChange = vm::onJoystick,
                    size = 170.dp,
                    ring = if (connected) colors.primary else colors.outline,
                    knob = if (connected) colors.primary else colors.outlineVariant,
                )
                Column(
                    modifier = Modifier.weight(1f),
                    verticalArrangement = Arrangement.spacedBy(6.dp),
                ) {
                    Readout(command)
                    Text("Limite: $limit %", style = MaterialTheme.typography.labelLarge)
                    Slider(
                        value = limit.toFloat(),
                        onValueChange = { vm.setLimit(it.toInt()) },
                        valueRange = 10f..100f,
                        steps = 8,
                    )
                    Button(
                        onClick = vm::stop,
                        enabled = connected,
                        colors = ButtonDefaults.buttonColors(containerColor = StopRed, contentColor = Color.White),
                        modifier = Modifier
                            .fillMaxWidth()
                            .height(64.dp),
                    ) {
                        Text("PARAR", fontSize = 22.sp, fontWeight = FontWeight.Bold)
                    }
                }
            }
        }
    }

    if (settingsOpen) {
        FeedDialog(
            config = config,
            onConnect = { mapVm.connect(it); settingsOpen = false },
            onSimulate = { mapVm.simulate(); settingsOpen = false },
            onStop = { mapVm.stop(); settingsOpen = false },
            onClear = { mapVm.clearMap(); settingsOpen = false },
            onDismiss = { settingsOpen = false },
        )
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

    Column(modifier = Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        Text(
            text = when (state) {
                LinkState.Idle -> "Bluetooth: desconectado"
                is LinkState.Connecting -> "Bluetooth: conectando em ${state.name}…"
                is LinkState.Connected -> "Bluetooth: conectado em ${state.name}"
                is LinkState.Failed -> state.message
            },
            style = MaterialTheme.typography.titleSmall,
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
private fun FeedBar(feed: FeedState, onOpen: () -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Text(
            text = when (feed) {
                FeedState.Off -> "Mapa: desligado"
                is FeedState.Connecting -> "Mapa: conectando em ${feed.broker}…"
                is FeedState.Live -> "Mapa: ao vivo de ${feed.broker}"
                FeedState.Simulating -> "Mapa: simulação"
                is FeedState.Failed -> feed.message
            },
            style = MaterialTheme.typography.titleSmall,
            color = when (feed) {
                is FeedState.Live, FeedState.Simulating -> MaterialTheme.colorScheme.primary
                is FeedState.Failed -> MaterialTheme.colorScheme.error
                else -> MaterialTheme.colorScheme.onBackground
            },
            modifier = Modifier.weight(1f),
        )
        OutlinedButton(onClick = onOpen) { Text("Mapa") }
    }
}

@Composable
private fun SonarRow(latest: Telemetry?) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        for ((name, label) in listOf("front" to "F", "rear" to "T", "left" to "E", "right" to "D")) {
            val d = latest?.sonar?.get(name)
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                Box(
                    Modifier
                        .size(10.dp)
                        .clip(RoundedCornerShape(5.dp))
                        .background(SONAR_COLORS.getValue(name))
                )
                Text(
                    if (d == null) "$label  --  " else "$label %.2f m".format(d),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 13.sp,
                )
            }
        }
        if (latest?.timeoutActive == true) {
            Text("TIMEOUT", color = MaterialTheme.colorScheme.error, fontWeight = FontWeight.Bold, fontSize = 13.sp)
        }
    }
}

@Composable
private fun FeedDialog(
    config: BrokerConfig,
    onConnect: (BrokerConfig) -> Unit,
    onSimulate: () -> Unit,
    onStop: () -> Unit,
    onClear: () -> Unit,
    onDismiss: () -> Unit,
) {
    var host by remember { mutableStateOf(config.host) }
    var port by remember { mutableStateOf(config.port.toString()) }
    var roverId by remember { mutableStateOf(config.roverId) }
    val portNumber = port.toIntOrNull()

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Telemetria do mapa") },
        text = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                OutlinedTextField(
                    value = host,
                    onValueChange = { host = it.trim() },
                    label = { Text("Broker (IP ou nome)") },
                    singleLine = true,
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Uri),
                )
                OutlinedTextField(
                    value = port,
                    onValueChange = { port = it.filter(Char::isDigit) },
                    label = { Text("Porta") },
                    singleLine = true,
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                )
                OutlinedTextField(
                    value = roverId,
                    onValueChange = { roverId = it.trim() },
                    label = { Text("ID do rover") },
                    singleLine = true,
                )
                Text("Tópico: rover/$roverId/telemetry", style = MaterialTheme.typography.bodySmall)
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(onClick = onSimulate) { Text("Simular") }
                    OutlinedButton(onClick = onClear) { Text("Limpar") }
                    OutlinedButton(onClick = onStop) { Text("Parar") }
                }
            }
        },
        confirmButton = {
            Button(
                onClick = { onConnect(BrokerConfig(host, portNumber ?: 1883, roverId)) },
                enabled = host.isNotEmpty() && portNumber != null && roverId.isNotEmpty(),
            ) { Text("Conectar") }
        },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Fechar") } },
    )
}

@Composable
private fun Readout(command: Command) {
    Column {
        Text("Linear  %+4d %%".format(command.linear), fontFamily = FontFamily.Monospace)
        Text("Rotação %+4d %%".format(command.rotation), fontFamily = FontFamily.Monospace)
    }
}
