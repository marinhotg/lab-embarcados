# Carrinho de Controle Remoto com Sensoriamento e Mapeamento 2D

PCS3858 — Laboratório de Sistemas Embarcados, Turma 1. Firmware ESP-IDF para
ESP32 e cliente de mapa 2D em Python.

## Organização

```
firmware/          projeto ESP-IDF
  main/
    config.h       pinos (Tabela 5) e constantes de calibração — único ponto de ajuste
    secrets.h      credenciais, fora do Git (copiar de secrets.h.example)
    main.c         inicialização e criação das tarefas
    state.[ch]     estado compartilhado, mutex e snapshot
    bt_command.c   Tarefa Bluetooth   — SPP, parser de 2 bytes, timeout
    sonar.c        Tarefa Sonar       — 4× HC-SR04 sequenciais, mediana, idade
    odometry.c     Tarefa Odom/IMU    — encoders por ISR, equações (1)–(6)
    mpu6050.[ch]   driver I2C da IMU
    control.c      Tarefa Controle    — segurança e ponte H (único escritor)
    telemetry.c    Tarefa MQTT        — Wi-Fi, esp-mqtt, JSON
  test/            ensaios de host, sem placa
client/
  map_client.py    mapa 2D em tempo real (paho-mqtt + matplotlib)
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

## Cliente de mapa

```sh
pip install -r client/requirements.txt
python client/map_client.py --broker 192.168.0.100 --id 01
```

## Ensaios de host

```sh
firmware/test/run_tests.sh
```

Conferem a cinemática diferencial contra a geometria analítica (reta de 2 m,
giro no lugar, arco de raio 0,5 m, passagem por ±π) e a validade do JSON de
telemetria. Não duplicam código: recortam os trechos de `main/` e compilam com
stubs, então acompanham qualquer mudança na fórmula ou no payload.

## Protocolo de comando (Tabela 7)

Dois bytes por pacote, via Bluetooth Classic SPP no dispositivo `Carrinho`.

| Comando              | Byte 1 | Byte 2                                        |
|----------------------|--------|-----------------------------------------------|
| Parada               | `0x00` | ignorado; zera ambas as referências            |
| Velocidade linear    | `0x01` | −100 a 100, percentual de `V_MAX_MPS`          |
| Velocidade de rotação| `0x02` | −100 a 100, percentual de `OMEGA_MAX_RAD_S`    |

Pacote inválido não renova o timeout. Sem pacote válido por 450 ms, o controle
aplica parada (RNF03 exige até 500 ms).

## Telemetria

Tópico `carrinho/{id}/telemetria`, QoS 0, sem retenção, a cada 200 ms. Esquema
na seção 4.6 do documento; `schema_versao` é 1. Dado inválido vai como `null`.

## Calibração

Tudo o que precisa de medição está em `firmware/main/config.h`, marcado. Os
valores atuais são estimativas iniciais, não medidas — a Aula 12 é a calibração
de raio efetivo, bitola, pulsos por volta e offsets.
