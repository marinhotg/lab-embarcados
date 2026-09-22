#!/usr/bin/env python3
"""Cliente de mapeamento 2D (RF12).

Assina o topico unico de telemetria, reconstroi a trajetoria e projeta as
leituras dos quatro ultrassonicos no plano.

Porte de Semana3/mqtt/scripts/calculate.py do projeto anterior, que fazia o
mesmo com rclpy e dois sensores. Mudou o transporte (ROS 2 -> MQTT + JSON),
o numero de sensores (2 -> 4, com offset e orientacao proprios) e, sobretudo,
a forma de ancorar a leitura no tempo: em vez de usar a pose corrente, cada
distancia e projetada sobre a pose que o carrinho tinha quando a medicao
aconteceu, obtida do historico por interpolacao a partir de sensores_idade_ms.
A odometria em si foi para o firmware; aqui so se desenha.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import threading

import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

# Geometria dos sensores no referencial do carrinho, espelhando as constantes
# de firmware/main/config.h. x aponta para a frente, y para a esquerda, angulo
# positivo anti-horario.  CALIBRAR junto com o firmware na Aula 12.
SENSORES = {
    "frente":   {"offset": (0.10, 0.00), "bearing": 0.0},
    "tras":     {"offset": (-0.10, 0.00), "bearing": math.pi},
    "esquerda": {"offset": (0.00, 0.07), "bearing": math.pi / 2},
    "direita":  {"offset": (0.00, -0.07), "bearing": -math.pi / 2},
}

CORES = {
    "frente": "tab:red",
    "tras": "tab:purple",
    "esquerda": "tab:green",
    "direita": "tab:orange",
}


class MapaCarrinho:
    def __init__(self) -> None:
        self._lock = threading.Lock()

        # Historico de pose: listas paralelas, ordenadas por tempo, para
        # permitir busca binaria na interpolacao.
        self._t_ms: list[int] = []
        self._poses: list[tuple[float, float, float]] = []

        self._trajetoria: list[tuple[float, float]] = []
        self._obstaculos: dict[str, list[tuple[float, float]]] = {
            nome: [] for nome in SENSORES
        }

        self._vistos: set[tuple[str, int]] = set()
        self._ultimo_seq: int | None = None
        self._ultimo_ts: int = -1
        self._estado = "aguardando telemetria"

    # -- recepcao ---------------------------------------------------------

    def on_message(self, _client, _userdata, msg) -> None:
        try:
            amostra = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            print(f"payload invalido: {exc}")
            return

        with self._lock:
            self._processar(amostra)

    def _processar(self, amostra: dict) -> None:
        ts = amostra.get("timestamp_ms")
        seq = amostra.get("seq")
        pose = amostra.get("pose")
        if ts is None or pose is None:
            return

        # timestamp_ms conta desde a inicializacao do carrinho. Se ele andou
        # para tras, houve reinicializacao: comeca uma sessao nova (secao 4.6).
        if ts < self._ultimo_ts:
            print("carrinho reinicializado, limpando o mapa")
            self._reset()

        self._ultimo_ts = ts

        if seq is not None and seq == self._ultimo_seq:
            return
        self._ultimo_seq = seq

        x, y, yaw = pose["x_m"], pose["y_m"], pose["yaw_rad"]
        self._t_ms.append(ts)
        self._poses.append((x, y, yaw))
        self._trajetoria.append((x, y))

        self._projetar(amostra, ts)

        seguranca = amostra.get("seguranca", {})
        if seguranca.get("timeout_ativo"):
            self._estado = "timeout de comando"
        elif seguranca.get("frenagem_automatica"):
            self._estado = "frenagem automatica"
        elif seguranca.get("frontal_invalido"):
            self._estado = "leitura frontal invalida"
        else:
            self._estado = "em operacao"

    def _projetar(self, amostra: dict, ts: int) -> None:
        distancias = amostra.get("sensores_m") or {}
        idades = amostra.get("sensores_idade_ms") or {}

        for nome, geometria in SENSORES.items():
            dist = distancias.get(nome)
            idade = idades.get(nome)

            # Dado invalido chega como null; sem idade nao da para ancorar.
            if dist is None or idade is None:
                continue

            t_medicao = ts - idade

            # A mesma leitura reaparece em varias amostras: o sonar atualiza a
            # cada 400 ms e a telemetria publica a cada 200 ms.
            chave = (nome, t_medicao)
            if chave in self._vistos:
                continue
            self._vistos.add(chave)

            pose = self._pose_em(t_medicao)
            if pose is None:
                continue

            px, py, yaw = pose
            ox, oy = geometria["offset"]

            # Posicao do sensor no mundo: offset rotacionado pelo yaw.
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            sx = px + ox * cos_y - oy * sin_y
            sy = py + ox * sin_y + oy * cos_y

            # Ponto detectado, na direcao para onde o sensor aponta.
            direcao = yaw + geometria["bearing"]
            self._obstaculos[nome].append(
                (sx + dist * math.cos(direcao), sy + dist * math.sin(direcao))
            )

    def _pose_em(self, t_ms: int) -> tuple[float, float, float] | None:
        """Pose interpolada no instante pedido, ou None se estiver fora do
        historico. Sem pose correspondente, a leitura e descartada."""
        if not self._t_ms:
            return None

        if t_ms <= self._t_ms[0]:
            return self._poses[0]
        if t_ms >= self._t_ms[-1]:
            return self._poses[-1]

        i = bisect.bisect_left(self._t_ms, t_ms)
        t0, t1 = self._t_ms[i - 1], self._t_ms[i]
        p0, p1 = self._poses[i - 1], self._poses[i]

        if t1 == t0:
            return p1

        f = (t_ms - t0) / (t1 - t0)
        x = p0[0] + f * (p1[0] - p0[0])
        y = p0[1] + f * (p1[1] - p0[1])

        # Interpolacao angular pelo menor arco, para nao girar 350 graus na
        # passagem por +-pi.
        delta = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
        yaw = p0[2] + f * delta

        return (x, y, yaw)

    def _reset(self) -> None:
        self._t_ms.clear()
        self._poses.clear()
        self._trajetoria.clear()
        self._vistos.clear()
        for pontos in self._obstaculos.values():
            pontos.clear()
        self._ultimo_seq = None

    # -- desenho ----------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "trajetoria": list(self._trajetoria),
                "obstaculos": {n: list(p) for n, p in self._obstaculos.items()},
                "pose": self._poses[-1] if self._poses else None,
                "estado": self._estado,
            }


def executar(broker: str, porta: int, carrinho_id: str) -> None:
    mapa = MapaCarrinho()
    topico = f"carrinho/{carrinho_id}/telemetria"

    cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    cliente.on_message = mapa.on_message
    cliente.on_connect = lambda c, *_: (
        print(f"conectado ao broker, assinando {topico}"),
        c.subscribe(topico),
    )
    cliente.connect(broker, porta, keepalive=30)
    cliente.loop_start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 9))

    linha_traj, = ax.plot([], [], "-", color="tab:blue", linewidth=1.5,
                          label="trajetoria")
    seta = ax.quiver([], [], [], [], color="tab:blue", scale=12)
    dispersao = {
        nome: ax.plot([], [], "x", color=CORES[nome], markersize=7,
                      linestyle="None", label=nome)[0]
        for nome in SENSORES
    }

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.axis("equal")
    ax.legend(loc="upper right")

    try:
        while plt.fignum_exists(fig.number):
            dados = mapa.snapshot()

            if dados["trajetoria"]:
                xs, ys = zip(*dados["trajetoria"])
                linha_traj.set_data(xs, ys)

            for nome, pontos in dados["obstaculos"].items():
                if pontos:
                    px, py = zip(*pontos)
                    dispersao[nome].set_data(px, py)

            if dados["pose"]:
                x, y, yaw = dados["pose"]
                seta.set_offsets([[x, y]])
                seta.set_UVC([math.cos(yaw)], [math.sin(yaw)])
                ax.set_title(
                    f"x={x:+.2f} m  y={y:+.2f} m  yaw={math.degrees(yaw):+.1f}"
                    f"   [{dados['estado']}]"
                )

            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw_idle()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cliente.loop_stop()
        cliente.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description="Mapa 2D do carrinho")
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--porta", type=int, default=1883)
    parser.add_argument("--id", dest="carrinho_id", default="01")
    args = parser.parse_args()

    executar(args.broker, args.porta, args.carrinho_id)


if __name__ == "__main__":
    main()
