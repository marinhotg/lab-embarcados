package br.usp.pcs3858.rover.telemetry

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.flow.flowOn
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken
import org.eclipse.paho.client.mqttv3.MqttCallbackExtended
import org.eclipse.paho.client.mqttv3.MqttClient
import org.eclipse.paho.client.mqttv3.MqttConnectOptions
import org.eclipse.paho.client.mqttv3.MqttMessage
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence

sealed interface FeedEvent {
    data object Connected : FeedEvent
    data class Lost(val reason: String) : FeedEvent
    data class Message(val payload: String) : FeedEvent
}

private const val RETRY_MS = 2000L

/* Subscribes to rover/{id}/telemetry and keeps trying every RETRY_MS while the
 * broker is unreachable, including before the first connection: the broker
 * lives on the rover, so it disappears whenever the ESP32 reboots. Paho's own
 * reconnect only covers drops after a first success, with a growing backoff. */
fun mqttFeed(host: String, port: Int, roverId: String): Flow<FeedEvent> = callbackFlow {
    val topic = "rover/$roverId/telemetry"
    val client = MqttClient("tcp://$host:$port", "rover-app-${System.currentTimeMillis()}", MemoryPersistence())

    client.setCallback(object : MqttCallbackExtended {
        override fun connectComplete(reconnect: Boolean, serverURI: String?) {
            runCatching { client.subscribe(topic, 0) }
            trySend(FeedEvent.Connected)
        }

        override fun connectionLost(cause: Throwable?) {
            trySend(FeedEvent.Lost("conexão perdida, tentando de novo"))
        }

        override fun messageArrived(topic: String?, message: MqttMessage) {
            trySend(FeedEvent.Message(String(message.payload)))
        }

        override fun deliveryComplete(token: IMqttDeliveryToken?) = Unit
    })

    val options = MqttConnectOptions().apply {
        isAutomaticReconnect = false
        isCleanSession = true
        connectionTimeout = 3
        keepAliveInterval = 5
    }

    val keeper = launch {
        while (isActive) {
            if (!client.isConnected) {
                try {
                    client.connect(options)
                } catch (e: Exception) {
                    trySend(FeedEvent.Lost("sem resposta, tentando de novo"))
                }
            }
            delay(RETRY_MS)
        }
    }

    awaitClose {
        keeper.cancel()
        runCatching { if (client.isConnected) client.disconnect() }
        runCatching { client.close() }
    }
}.flowOn(Dispatchers.IO)
