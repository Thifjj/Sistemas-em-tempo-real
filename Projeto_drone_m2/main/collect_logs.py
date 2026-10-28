"""Collect logs: tcp_client.py/udp_server.py --host IP --experiment NAME.
Use --clock-synchronized only after verifying the PC clock synchronization.
"""
import argparse
import json
import socket
import threading
import time


def packet_record(protocol, direction, data, experiment, clock_synchronized):
    return {"direction": direction, "epoch_ms": time.time_ns() // 1_000_000,
            "protocol": protocol, "experiment": experiment, "bytes": len(data),
            "pc_clock_synchronized": clock_synchronized,
            "mensagem": data.decode(errors="replace").rstrip()}


BATCH_FIELDS = ("task", "seq", "event_id", "source", "evento_us", "inicio_us", "fim_us",
                "touch_channel", "run", "type", "command_seq", "pc_send_ms", "received_us",
                "received_epoch_ms", "lock_wait_us", "off_cpu_total_us", "switches_total")


def expand_messages(data):
    try:
        record=json.loads(data)
    except (ValueError, UnicodeDecodeError):
        return [data]
    if not isinstance(record, dict) or record.get("kind") != "batch":
        return [data]
    common={key:value for key,value in record.items() if key not in ("rows", "kind")}
    result=[]
    for row in record.get("rows", []):
        if not isinstance(row,list) or len(row) != len(BATCH_FIELDS):
            raise ValueError("Invalid event batch row")
        event=dict(common, kind="event", **dict(zip(BATCH_FIELDS,row)))
        result.append(json.dumps(event).encode())
    return result


def main(protocol):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=5000 if protocol == "TCP" else 6010)
    parser.add_argument("--experiment", required=True)
    parser.add_argument("--output")
    parser.add_argument("--clock-synchronized", action="store_true")
    args = parser.parse_args()
    pending = {}
    lock = threading.Lock()
    output = args.output or f"log_{args.experiment}_{protocol.lower()}.jsonl"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM if protocol == "TCP" else socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    if protocol == "TCP":
        sock.settimeout(5)
        sock.connect((args.host, args.port))
        sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        sock.settimeout(None)
    else:
        sock.bind(("0.0.0.0", args.port))
    if protocol == "UDP":
        sock.sendto(f"0 {time.time_ns() // 1_000_000} HELLO\n".encode(), (args.host, args.port))
    def save(direction, data, now=None):
        record = packet_record(protocol, direction, data, args.experiment, args.clock_synchronized)
        with lock:
            if direction == "received" and data.startswith(b"ACK "):
                parts = data.split()
                try:
                    seq = int(parts[1])
                    started = pending.pop(seq, None)
                    if started is not None:
                        record["rtt_ms"] = ((now or time.monotonic_ns())-started)/1_000_000
                except (ValueError, IndexError):
                    pass
            with open(output, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(record) + "\n")
        if data.startswith(b"ACK "):
            print(record)
    def receive():
        try:
            if protocol == "TCP":
                with sock.makefile("rb") as stream:
                    for line in stream:
                        arrived=time.monotonic_ns()
                        for event in expand_messages(line):
                            save("received", event, arrived)
            else:
                while True:
                    data, origin = sock.recvfrom(8192)
                    if origin[0] == socket.gethostbyname(args.host):
                        arrived=time.monotonic_ns()
                        for event in expand_messages(data):
                            save("received", event, arrived)
        except OSError as error:
            print(f"Recepcao encerrada: {error}")
    threading.Thread(target=receive, daemon=True).start()
    try:
        seq = 0
        while True:
            command = input("NAV, FS ou SAIR: ").strip().upper()
            if command == "SAIR":
                break
            if command not in ("NAV", "FS"):
                continue
            seq += 1
            data = f"{seq} {time.time_ns() // 1_000_000} {command}\n".encode()
            with lock:
                pending[seq] = time.monotonic_ns()
            save("sent", data)
            if protocol == "TCP":
                sock.sendall(data)
            else:
                sock.sendto(data, (args.host, args.port))
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        sock.close()
