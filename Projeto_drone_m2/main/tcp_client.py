"""
tcp_client.py
Exemplo para disciplina de Sistemas em Tempo Real
Prof. Felipe Viel
Curso de Engenharia de Computação da Univali
Exemplo de Cliente utilizando protocolo de transporte TCP.
A ESP32 será o servidor e essa aplicação deverá se conectar a ela usando IP gerado no log do terminal.
o irá enviar pacotes para o IP e porta indicadas.
A porta, que nesse exemplo é 5000, pode ser alterada para qualquer número dentro do intervalo de 2^16.
    - Deve ser igual em ambos os computadores
"""
import socket, json

ESP_IP = "10.27.142.37"  # coloque o IP do seu ESP32
PORT   = 5000

with socket.create_connection((ESP_IP, PORT), timeout=5) as s:
    print(s.recv(1024).decode())   # mensagem de boas-vindas
    for msg in ["ping", "temperatura?", "bye"]:
        s.sendall((msg+"\n").encode())
        data = s.recv(1024).decode()
        print("ESP>", data)
        try:
            print("JSON:", json.loads(data))
        except json.JSONDecodeError:
            pass