import socket
import threading

ESP_IP = "172.16.0.7"  # Substitua pelo IP da ESP32
UDP_PORT = 6010
TCP_PORT = 5000

protocolo = input("Protocolo (tcp/udp): ").strip().lower()

if protocolo not in ("tcp", "udp"):
    raise SystemExit("Protocolo invalido")

if protocolo == "tcp":
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ESP_IP, TCP_PORT))
else:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.settimeout(0.5)
executando = threading.Event()
executando.set()


def receber_logs():
    buffer = ""

    while executando.is_set():
        try:
            if protocolo == "tcp":
                dados = sock.recv(4096)

                if not dados:
                    print("\nESP32 desconectada")
                    executando.clear()
                    break

                buffer += dados.decode(errors="replace")

                while "\n" in buffer:
                    linha, buffer = buffer.split("\n", 1)
                    if linha.strip():
                        print(f"\nESP32: {linha.strip()}")

            else:
                dados, endereco = sock.recvfrom(4096)
                print(f"\nESP32: {dados.decode(errors='replace').strip()}")

        except socket.timeout:
            continue

        except OSError:
            executando.clear()
            break


threading.Thread(target=receber_logs, daemon=True).start()

print("Monitor conectado!")
print("Comandos: fs | nav | sair")

try:
    while executando.is_set():
        comando = input("Comando: ").strip().lower()

        if comando == "sair":
            break

        if comando not in ("fs", "nav"):
            print("Comando invalido")
            continue

        if protocolo == "tcp":
            sock.sendall((comando + "\n").encode())
        else:
            sock.sendto(comando.encode(), (ESP_IP, UDP_PORT))

except (KeyboardInterrupt, EOFError):
    pass

finally:
    executando.clear()
    sock.close()
    print("Monitor encerrado")