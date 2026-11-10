package br.usp.pcs3858.rover.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import java.io.IOException
import java.io.OutputStream
import java.util.UUID
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext

/* SPP link with the protocol of client/bt_control.py. */
class RoverLink {

    private val lock = Mutex()
    private var socket: BluetoothSocket? = null
    private var output: OutputStream? = null

    @SuppressLint("MissingPermission")
    suspend fun connect(device: BluetoothDevice) = withContext(Dispatchers.IO) {
        lock.withLock {
            closeLocked()
            val s = device.createRfcommSocketToServiceRecord(SPP_UUID)
            try {
                s.connect()
            } catch (e: IOException) {
                runCatching { s.close() }
                throw e
            }
            socket = s
            output = s.outputStream
        }
    }

    /* Sent every period: each valid packet renews the firmware timeout. */
    suspend fun send(linear: Int, rotation: Int) = write(
        byteArrayOf(
            CMD_LINEAR, linear.coerceIn(-100, 100).toByte(),
            CMD_ROTATION, rotation.coerceIn(-100, 100).toByte(),
        )
    )

    suspend fun stop() = write(byteArrayOf(CMD_STOP, 0))

    suspend fun close() = lock.withLock { closeLocked() }

    private suspend fun write(bytes: ByteArray) = withContext(Dispatchers.IO) {
        lock.withLock {
            val out = output ?: throw IOException("not connected")
            out.write(bytes)
            out.flush()
        }
    }

    private fun closeLocked() {
        runCatching { socket?.close() }
        socket = null
        output = null
    }

    private companion object {
        val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        const val CMD_STOP: Byte = 0x00
        const val CMD_LINEAR: Byte = 0x01
        const val CMD_ROTATION: Byte = 0x02
    }
}
