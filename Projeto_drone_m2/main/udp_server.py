"""Recebe logs UDP e envia comandos a ESP32 pelo mesmo socket."""
import socket
import threading
import time
import json

ESP_IP = "172.16.0.2"  # Substitua pelo IP mostrado no monitor serial da ESP32.
PORT = 6010
PROTOCOLO = "UDP"
pendentes = {}
sequencia = 0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))


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
    # Os eventos chegam continuamente; mostrá-los ocultaria o prompt de entrada.
    if direcao != "recebido" or mensagem.startswith("ACK "):
        print(registro)


def receber():
    while True:
        dados, origem = sock.recvfrom(2048)
        registrar("recebido", dados.decode(errors="replace").rstrip())


threading.Thread(target=receber, daemon=True).start()
print("Logs recebidos são gravados em log_udp.jsonl. Digite um comando abaixo.")
while True:
    comando = input("Comando (NAV, FS ou sair): ").strip().upper()
    if comando == "SAIR":
        break
    sequencia += 1
    mensagem = f"{sequencia} {comando}"
    pendentes[sequencia] = time.monotonic_ns()
    registrar("enviado", mensagem, sequencia)
    sock.sendto(mensagem.encode(), (ESP_IP, PORT))
sock.close()
