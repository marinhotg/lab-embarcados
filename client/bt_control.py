#!/usr/bin/env python3
"""Controle Bluetooth do carrinho (Tabela 7), por socket RFCOMM nativo.

Modos:

    ./bt_control.py                     teclado, envia a cada 100 ms
    ./bt_control.py --hex "01 32"       manda bytes crus e sai
    ./bt_control.py --teste-timeout     manda um pacote e cala, para ver o
                                        timeout de 450 ms disparar no log

Antes do primeiro uso, parear uma vez:

    bluetoothctl
    scan on                 # espere aparecer "Carrinho"
    pair E4:65:B8:1F:C1:92
    trust E4:65:B8:1F:C1:92
    scan off
"""

from __future__ import annotations

import argparse
import socket
import sys
import termios
import time
import tty

MAC_PADRAO = "E4:65:B8:1F:C1:92"

CMD_PARADA = 0x00
CMD_LINEAR = 0x01
CMD_ROTACAO = 0x02

PERIODO_S = 0.1   # o documento especifica um envio a cada 100 ms
PASSO = 10        # incremento por tecla, em pontos percentuais

AJUDA = """
  w / s   avanco / re          (passo de 10%)
  a / d   girar esq / dir      (passo de 10%)
  espaco  parada (0x00), zera as duas referencias
  0       zera as referencias sem mandar parada
  q       sair
"""


def conectar(mac: str, canal: int | None) -> socket.socket:
    canais = [canal] if canal else range(1, 6)
    ultimo_erro = None

    for c in canais:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                             socket.BTPROTO_RFCOMM)
        sock.settimeout(10)
        try:
            sock.connect((mac, c))
            print(f"conectado a {mac}, canal RFCOMM {c}")
            return sock
        except OSError as exc:
            ultimo_erro = exc
            sock.close()

    print(f"nao foi possivel conectar a {mac}: {ultimo_erro}", file=sys.stderr)
    print("pareou com bluetoothctl? veja o cabecalho deste arquivo.",
          file=sys.stderr)
    sys.exit(1)


def enviar(sock: socket.socket, identificador: int, valor: int) -> None:
    """Um pacote de dois bytes: identificador + inteiro de 8 bits com sinal."""
    sock.send(bytes([identificador, valor & 0xFF]))


def modo_hex(sock: socket.socket, texto: str) -> None:
    dados = bytes(int(b, 16) for b in texto.replace(",", " ").split())
    sock.send(dados)
    print("enviado:", " ".join(f"{b:02X}" for b in dados))


def modo_teste_timeout(sock: socket.socket) -> None:
    print("mandando 01 32 (avanco 50%) e ficando em silencio.")
    print("o log da ESP32 deve mostrar a parada por timeout em ~450 ms.")
    enviar(sock, CMD_LINEAR, 50)
    time.sleep(2.0)
    print("pronto - confira o monitor serial.")


def ler_tecla() -> str | None:
    """Le uma tecla sem bloquear, com o terminal em modo cru."""
    import select
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def modo_teclado(sock: socket.socket) -> None:
    v = 0
    w = 0
    print(AJUDA)

    fd = sys.stdin.fileno()
    antigo = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        proximo = time.monotonic()

        while True:
            tecla = ler_tecla()

            if tecla == "q":
                enviar(sock, CMD_PARADA, 0)
                print("\nparada enviada, saindo.")
                return
            elif tecla == " ":
                v = w = 0
                enviar(sock, CMD_PARADA, 0)
            elif tecla == "0":
                v = w = 0
            elif tecla == "w":
                v = min(100, v + PASSO)
            elif tecla == "s":
                v = max(-100, v - PASSO)
            elif tecla == "a":
                w = min(100, w + PASSO)
            elif tecla == "d":
                w = max(-100, w - PASSO)

            agora = time.monotonic()
            if agora >= proximo:
                # Os dois comandos a cada 100 ms (secao 4.6); tambem mantem o
                # timeout vivo.
                enviar(sock, CMD_LINEAR, v)
                enviar(sock, CMD_ROTACAO, w)
                proximo = agora + PERIODO_S
                print(f"\r  v={v:+4d}%   w={w:+4d}%   "
                      f"[01 {v & 0xFF:02X}] [02 {w & 0xFF:02X}]   ",
                      end="", flush=True)

            time.sleep(0.005)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, antigo)


def main() -> None:
    p = argparse.ArgumentParser(description="Controle Bluetooth do carrinho")
    p.add_argument("--mac", default=MAC_PADRAO)
    p.add_argument("--canal", type=int, default=None,
                   help="canal RFCOMM; por padrao tenta de 1 a 5")
    p.add_argument("--hex", dest="hexa",
                   help='bytes crus, ex: --hex "01 32"')
    p.add_argument("--teste-timeout", action="store_true")
    args = p.parse_args()

    sock = conectar(args.mac, args.canal)
    try:
        if args.hexa:
            modo_hex(sock, args.hexa)
        elif args.teste_timeout:
            modo_teste_timeout(sock)
        else:
            modo_teclado(sock)
    except KeyboardInterrupt:
        try:
            enviar(sock, CMD_PARADA, 0)
        except OSError:
            pass
        print("\ninterrompido, parada enviada.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
