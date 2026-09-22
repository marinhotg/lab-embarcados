# Carrinho de Controle Remoto com Sensoriamento e Mapeamento 2D

PCS3858 — Laboratório de Sistemas Embarcados, Turma 1. Firmware ESP-IDF para
ESP32 e cliente Bluetooth em Python.

## Organização

```
firmware/          projeto ESP-IDF
  main/
    config.h       pinos e constantes de calibração — único ponto de ajuste
    secrets.h      credenciais, fora do Git (copiar de secrets.h.example)
    main.c         inicialização e criação das tarefas
    state.[ch]     estado compartilhado, mutex e snapshot
    bt_command.c   Bluetooth   — SPP, parser de 2 bytes, timeout
    sonar.c        sonar       — 4× HC-SR04 sequenciais, mediana, idade
    odometry.c     odometria   — encoders por ISR, fusão com giroscópio
    mpu6050.[ch]   driver I2C da IMU
    control.c      controle    — segurança e ponte H (único escritor)
    telemetry.c    telemetria  — Wi-Fi, esp-mqtt, JSON
  test/            ensaios de host, sem placa
client/
  bt_control.py    controle Bluetooth por teclado e ensaios de bancada
```

## Requisitos

ESP-IDF **v5.2 ou mais recente** — `bt_command.c` usa `esp_bt_gap_set_device_name()`
e `mpu6050.c` usa o driver `i2c_master`, ambos introduzidos nessa versão.

## Compilar e gravar

```sh
cp firmware/main/secrets.h.example firmware/main/secrets.h
$EDITOR firmware/main/secrets.h          # SSID, senha e URI do broker

cd firmware
idf.py set-target esp32
idf.py build flash monitor
```

`sdkconfig.defaults` já traz o que o projeto precisa: partição grande (Bluetooth
Classic e Wi-Fi juntos não cabem no padrão de 1 MB), coexistência de rádio
ligada e tick do FreeRTOS em 1 ms.

## Controle Bluetooth

```sh
python client/bt_control.py              # teclado: w/s avança e ré, a/d gira
python client/bt_control.py --motor-test # confere a fiação da ponte H
```

O cabeçalho de `bt_control.py` lista os outros modos (`--hex`,
`--timeout-test`, `--ramp`, `--power-test`) e como parear com `bluetoothctl`.

## Ensaios de host

```sh
firmware/test/run_tests.sh
```

Conferem a cinemática diferencial contra a geometria analítica (reta de 2 m,
giro no lugar, arco de raio 0,5 m, passagem por ±π) e a validade do JSON de
telemetria. Não duplicam código: recortam os trechos de `main/` e compilam com
stubs, então acompanham qualquer mudança na fórmula ou no payload.

## Protocolo de comando

Dois bytes por pacote, via Bluetooth Classic SPP no dispositivo `Rover`.

| Comando              | Byte 1 | Byte 2                                        |
|----------------------|--------|-----------------------------------------------|
| Parada               | `0x00` | ignorado; zera ambas as referências            |
| Velocidade linear    | `0x01` | −100 a 100, percentual de `V_MAX_MPS`          |
| Velocidade de rotação| `0x02` | −100 a 100, percentual de `OMEGA_MAX_RAD_S`    |

Pacote inválido não renova o timeout. Sem pacote válido por 450 ms, o controle
aplica parada (o requisito é parar em até 500 ms).

## Telemetria

Tópico `rover/{id}/telemetry`, QoS 0, sem retenção, a cada 200 ms.
`schema_version` é 1; dado inválido vai como `null`.

```json
{
  "schema_version": 1, "seq": 120, "timestamp_ms": 24000,
  "pose": {"x_m": 1.24, "y_m": 0.58, "yaw_rad": 0.31},
  "speed_m_s": 0.18,
  "wheel_speed_m_s": {"left": 0.16, "right": 0.20},
  "sonar_m": {"front": 0.87, "rear": 1.45, "left": 0.63, "right": 0.71},
  "sonar_age_ms": {"front": 300, "rear": 200, "left": 100, "right": 0},
  "imu": {
    "accel_m_s2": {"x": 0.02, "y": 0.01, "z": 9.81},
    "gyro_rad_s": {"x": 0.00, "y": 0.00, "z": 0.16},
    "age_ms": 0, "yaw_fused_rad": 0.31, "yaw_odom_rad": 0.29, "valid": true
  },
  "safety": {"timeout_active": false, "auto_brake": false, "front_invalid": false}
}
```

## Calibração

Tudo o que precisa de medição está em `firmware/main/config.h`. Os valores
atuais são estimativas iniciais, não medidas — a Aula 12 é a calibração de raio
efetivo, bitola, pulsos por volta e offsets.
