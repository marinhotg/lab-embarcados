package br.usp.pcs3858.rover.telemetry

import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.awaitClose
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

/* Subscribes to rover/{id}/telemetry. Paho reconnects on its own after a drop;
 * every (re)connection subscribes again. */
fun mqttFeed(host: String, port: Int, roverId: String): Flow<FeedEvent> = callbackFlow {
    val topic = "rover/$roverId/telemetry"
    val client = MqttClient("tcp://$host:$port", "rover-app-${System.currentTimeMillis()}", MemoryPersistence())

    client.setCallback(object : MqttCallbackExtended {
        override fun connectComplete(reconnect: Boolean, serverURI: String?) {
            runCatching { client.subscribe(topic, 0) }
            trySend(FeedEvent.Connected)
        }

        override fun connectionLost(cause: Throwable?) {
            trySend(FeedEvent.Lost(cause?.message ?: "conexão perdida"))
        }

        override fun messageArrived(topic: String?, message: MqttMessage) {
            trySend(FeedEvent.Message(String(message.payload)))
        }

        override fun deliveryComplete(token: IMqttDeliveryToken?) = Unit
    })

    val options = MqttConnectOptions().apply {
        isAutomaticReconnect = true
        isCleanSession = true
        connectionTimeout = 5
        keepAliveInterval = 10
    }

    try {
        client.connect(options)
    } catch (e: Exception) {
        trySend(FeedEvent.Lost(e.message ?: "broker não respondeu"))
        close()
    }

    awaitClose {
        runCatching { if (client.isConnected) client.disconnect() }
        runCatching { client.close() }
    }
}.flowOn(Dispatchers.IO)
