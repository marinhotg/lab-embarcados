# Montagem para o ensaio de potência

Pinos conforme `firmware/main/config.h`. Se mudar lá, mude aqui — o
`firmware/test/run_tests.sh` confere que as duas listas batem.

## Sinais: LOLIN32 → Ponte H Keyes L298

| LOLIN32 | Keyes | Função |
|---|---|---|
| GPIO 25 | ENA | PWM do canal A (lado esquerdo) |
| GPIO 27 | IN1 | sentido A |
| GPIO 26 | IN2 | sentido A |
| GPIO 13 | ENB | PWM do canal B (lado direito) |
| GPIO 32 | IN3 | sentido B |
| GPIO 33 | IN4 | sentido B |
| GND | GND | **terra comum, obrigatório** |

Tire os jumpers de ENA/ENB se a placa veio com eles — presos em 5 V o canal
fica sempre habilitado e o PWM não tem efeito.

## Sinais: LOLIN32 → sensores

| LOLIN32 | Componente | Observação |
|---|---|---|
| GPIO 14 | HC-SR04 frente, Trig | |
| GPIO 18 | HC-SR04 frente, Echo | **divisor de nível** |
| GPIO 4 | HC-SR04 trás, Trig | |
| GPIO 19 | HC-SR04 trás, Echo | **divisor de nível** |
| GPIO 5 | HC-SR04 esquerda, Trig | |
| GPIO 36 | HC-SR04 esquerda, Echo | **divisor de nível** |
| GPIO 23 | HC-SR04 direita, Trig | |
| GPIO 39 | HC-SR04 direita, Echo | **divisor de nível** |
| GPIO 34 | encoder esquerdo | pull-up externo 10 kΩ para 3,3 V |
| GPIO 35 | encoder direito | pull-up externo 10 kΩ para 3,3 V |
| GPIO 21 | MPU6050 SDA | |
| GPIO 22 | MPU6050 SCL | |

O echo do HC-SR04 sai em 5 V e o ESP32 **não é tolerante a 5 V**. Divisor em
cada um dos quatro:

```
Echo (5 V) ──[ 1 kΩ ]──┬── GPIO
                       │
                     [ 2 kΩ ]
                       │
                      GND
```

Sem resistores em mãos, alimente os HC-SR04 em 3,3 V: o alcance cai mas o pino
fica seguro. Para o ensaio de potência isso subestima o consumo dos sensores em
poucos miliampères — irrelevante diante dos motores.

GPIO 34 e 35 são input-only e não têm pull-up interno. O externo não é opcional.

## Alimentação

```
                  ┌─────────────────► VMS da Keyes ──► 4 motores
  Fonte CC 7,4 V ─┤
   (limite 3 A)   └─ GND ──────┬────► GND da Keyes
                               │
                               └────► GND da LOLIN32   (terra comum)

  USB do notebook ─────────────────► LOLIN32 (lógica 3,3 V)
                                        │
                                        └─► 3,3 V para IMU e encoders

  Borne 5 V da Keyes ──────────────► Vcc dos 4 HC-SR04
```

O 78M05 da Keyes gera os 5 V a partir do VMS, então **não alimente o borne 5 V
por fora** enquanto o VMS estiver ligado.

A ESP fica no USB e os motores na fonte: duas fontes separadas, e é por isso que
o GND comum é obrigatório — sem ele os dois lados não têm referência.

Nunca ligue a fonte no conector JST branco da LOLIN32. Aquilo é entrada de LiPo
1S com carregador; 7,4 V queima a placa.

## Por que 7,4 V

O L298 é ponte Darlington bipolar e perde 1,8–2 V entre os dois transistores em
condução. Com 5 V no VMS sobram 3 V no motor, abaixo do nominal — medimos em
22/09/2026 que a roda só destravava com 79% de duty. Com 7,4 V sobram 5,4 V e o
destrave deve cair para 30–40%.

## Ordem de energização

1. Fonte **desligada**, USB desconectado.
2. Sinais, terra comum, divisores e pull-ups.
3. Motores nos bornes MOTORA e MOTORB.
4. USB na ESP. Ela sobe parada: o timeout já nasce ativo.
5. Fonte em 7,4 V com limite de 3 A.

Entre o reset e o `control_start()` passam ~700 ms com os GPIOs de sentido
flutuando. Com o VMS já energizado isso pode dar um tranco nos motores — por
isso a fonte entra por último e sai primeiro.

## O ensaio

```sh
cd client && ./bt_control.py --power-test
```

Carrinho **no chão**, preso pelos fios da fonte. Rodas com carga é o que torna a
medida representativa.

| Fase | Duração | O que medir |
|---|---|---|
| Base | 30 s | corrente com ESP, rádios e sensores, motores parados |
| Avanço 50% | 20 s | tração em regime |
| Avanço 100% | 20 s | tração máxima |
| Giro 100% | 20 s | os dois lados opostos, pior caso de atrito |
| 8 arranques | 16 s | **pico** — é o que dimensiona a powerbank |

`tração = (avanço 100%) − (base)`.

A base só vale se os sensores estiverem fisicamente ligados. Sem eles a medida
exclui ~105 mA.
