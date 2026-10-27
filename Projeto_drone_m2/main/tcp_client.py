"""Cliente TCP: recebe logs e envia NAV ou FS (uma linha por comando)."""
import socket
import threading
import time
import json

ESP_IP = "10.27.142.37"  # Substitua pelo IP mostrado no monitor serial da ESP32.
PORT = 5000
PROTOCOLO = "TCP"
pendentes = {}
sequencia = 0


def registrar(direcao, mensagem, seq=None):
    # Relogio civil permite comparar dispositivos; monotonic calcula RTT local.
    registro = {"direcao": direcao, "epoch_ms": time.time_ns() // 1_000_000,
                "protocolo": PROTOCOLO, "mensagem": mensagem, "seq": seq}
    if direcao == "recebido" and mensagem.startswith("ACK "):
        partes = mensagem.split()
        if len(partes) >= 2 and partes[1].isdigit():
            registro["seq"] = int(partes[1])
            inicio = pendentes.pop(registro["seq"], None)
            if inicio is not None:
                registro["rtt_ms"] = (time.monotonic_ns() - inicio) / 1_000_000
    with open("log_" + PROTOCOLO.lower() + ".jsonl", "a", encoding="utf-8") as arquivo:
        arquivo.write(json.dumps(registro, ensure_ascii=False) + "\n")
    print(registro)


def receber(sock):
    # TCP e um fluxo: makefile agrupa os fragmentos ate a quebra de linha.
    with sock.makefile("r", encoding="utf-8") as linhas:
        for linha in linhas:
            registrar("recebido", linha.rstrip())


with socket.create_connection((ESP_IP, PORT), timeout=5) as conexao:
    conexao.settimeout(None)
    threading.Thread(target=receber, args=(conexao,), daemon=True).start()
    while True:
        comando = input("Comando (NAV, FS ou sair): ").strip().upper()
        if comando == "SAIR":
            break
        sequencia += 1
        mensagem = f"{sequencia} {comando}"
        pendentes[sequencia] = time.monotonic_ns()
        registrar("enviado", mensagem, sequencia)
        conexao.sendall((mensagem + "\n").encode())
