"""Same 60-second drone scenario, five repetitions, one firmware configuration."""
import argparse
import json
from pathlib import Path
import select
import socket
import time

from collect_logs import packet_record, expand_messages
from summarize_experiments import summarize_run, write_tables

# Navigation/telemetry first; manual emergency, then automatic emergency backup.
SCENARIO = sorted(
    [(float(t), "command", "NAV") for t in (2, 4, 6, 10, 12, 14, 18, 20, 22, 24, 26, 28, 30, 32, 34)]
    + [(8., "command", "T4"), (16., "command", "T9"),
       (36., "command", "T3"), (40., "command", "T0"),
       (48., "command", "FS")]
)
TASKS = {"FUS_IMU", "CTRL_ATT", "NAV_PLAN", "FS_TASK", "MONITOR_TASK", "TIME_TASK"}


class Session:
    def __init__(self, args, path):
        self.args = args
        self.address = (socket.gethostbyname(args.host), 5000 if args.protocol == "TCP" else 6010)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM if args.protocol == "TCP" else socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.sock.settimeout(10)
        if args.protocol == "TCP":
            self.sock.connect(self.address)
            self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        else:
            self.sock.bind(("0.0.0.0", 6010))
        self.buffer = b""
        self.seq = 0
        self.records = []
        self.file = path.open("x", encoding="utf-8")

    def save(self, direction, data, **extra):
        record = packet_record(self.args.protocol, direction, data,
                               self.args.configuration, self.args.clock_synchronized)
        record.update(extra)
        self.file.write(json.dumps(record) + "\n")
        self.file.flush()
        self.records.append(record)
        return record

    def receive(self, timeout):
        if not select.select([self.sock], [], [], max(0, timeout))[0]:
            return []
        if self.args.protocol == "UDP":
            data, origin = self.sock.recvfrom(8192)
            if origin[0] != self.address[0]:
                return []
            lines = [data]
        else:
            data = self.sock.recv(8192)
            if not data:
                raise RuntimeError("Conexao TCP encerrada durante o ensaio")
            self.buffer += data
            *lines, self.buffer = self.buffer.split(b"\n")
            lines = [line + b"\n" for line in lines]
        arrived = time.monotonic_ns()
        return [self.save("received", event, arrival_mono_ns=arrived)
                for line in lines for event in expand_messages(line)]

    def command(self, action):
        self.seq += 1
        data = f"{self.seq} {time.time_ns() // 1_000_000} {action}\n".encode()
        sent = time.monotonic_ns()
        self.save("sent", data, command_seq=self.seq)
        if self.args.protocol == "TCP":
            self.sock.sendall(data)
        else:
            self.sock.sendto(data, self.address)
        hello = action == "HELLO" and self.args.protocol == "UDP"
        retry_safe = self.args.protocol == "UDP"
        deadline = time.monotonic() + (60 if action in ("HELLO", "TOUCHON", "TOUCHOFF", "TOUCHGET") and retry_safe else (60 if self.args.protocol == "TCP" else 10))
        retries=0
        retry_at = time.monotonic() + 1
        informed=False
        while time.monotonic() < deadline:
            if self.args.protocol == "TCP" and not informed and time.monotonic_ns()-sent>1000000000:
                print(f"Aguardando ACK TCP de {action} (limite 60 s; comando nao sera reenviado)...",flush=True)
                informed=True
            if retry_safe and time.monotonic() >= retry_at:
                self.sock.sendto(data, self.address)
                retries+=1
                retry_at = time.monotonic() + 1
            for record in self.receive(min(.2, deadline-time.monotonic())):
                parts = record["mensagem"].split()
                if len(parts) > 2 and parts[0] == "ACK" and parts[1] == str(self.seq):
                    # Persist RTT separately so raw arrival timestamps stay unchanged.
                    self.save("measurement", b"", command_seq=self.seq, action=action,
                              rtt_ms=(record["arrival_mono_ns"]-sent)/1e6, retries=retries,
                              rtt_kind="with_retry" if retries else "without_retry")
                    if "OK" not in parts:
                        raise RuntimeError(f"Comando rejeitado: {record['mensagem']}")
                    if "TOUCH" in parts:
                        index=parts.index("TOUCH")+1
                        values=[int(value) for value in parts[index:]]
                        if len(values) != 9: raise RuntimeError("Resumo TOUCHGET invalido")
                        return {channel: {"latency_ms": values[1+i]/1000, "event_us": values[5+i]}
                                for i,channel in enumerate((4,9,0,3)) if values[0] & (1 << channel)}
                    return int(parts[parts.index("RUN")+1]) if "RUN" in parts else None
        if hello:
            raise RuntimeError(f"Sem resposta UDP de {self.address[0]}:6010 em 60 s. "
                               "Confira IP obtido na serial, firmware UDP, reinicios e bloqueio da rede/firewall.")
        raise RuntimeError(f"Sem ACK para {action}; ensaio incompleto apos aguardar confirmacao")

    def close(self):
        self.sock.close()
        self.file.close()


def decode(record):
    try:
        return json.loads(record["mensagem"])
    except (KeyError, ValueError, TypeError):
        return {}


def validate(record, args):
    expected = (160, 1) if args.configuration == "melhor" else (80, 0)
    actual = (record.get("frequency_mhz"), record.get("preemption"))
    if actual != expected or record.get("slicing") != 1 or record.get("protocol") != args.protocol:
        raise RuntimeError(f"Firmware diferente do solicitado: MHz/preempcao={actual}, "
                           f"slicing={record.get('slicing')}, protocolo={record.get('protocol')}; "
                           f"esperado {expected}, slicing=1, {args.protocol}")
    if record.get("policy") != "RM":
        raise RuntimeError("Firmware sem prioridades RM do experimento")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--protocol", choices=["TCP", "UDP"], required=True)
    parser.add_argument("--configuration", choices=["melhor", "pior"], required=True)
    parser.add_argument("--output", type=Path, default=Path("resultados"))
    parser.add_argument("--clock-synchronized", action="store_true")
    args = parser.parse_args()
    destination = args.output / f"{args.configuration}_{args.protocol.lower()}_{time.time_ns()}"
    destination.mkdir(parents=True)
    session = None
    run=None
    repetition=0
    first=0
    try:
        session = Session(args, destination / "raw.jsonl")
        session.command("HELLO")
        print("Conferindo firmware e aguardando sincronizacao SNTP (ate 90 s)...", flush=True)
        deadline = time.monotonic()+90
        ready = False
        while time.monotonic() < deadline and not ready:
            for outer in session.receive(.2):
                record = decode(outer)
                if record.get("kind") in ("event", "metric"):
                    validate(record, args)
                if record.get("kind") in ("event", "metric") and record.get("sntp"):
                    ready = True
        if not ready:
            raise RuntimeError("Sem evento com SNTP sincronizado; verifique Wi-Fi, IP do PC e protocolo")
        session.command("TOUCHOFF")
        first_physical = len(session.records)
        physical_run = session.command("START")
        print("TESTE FISICO: solte os quatro eletrodos. Toque GPIO13, GPIO32, GPIO4 e GPIO15 apos o aviso; nao precisam acionar no mesmo instante.", flush=True)
        input("Com todos soltos, pressione Enter para armar: ")
        session.command("TOUCHON")
        print("TOQUE OS QUATRO AGORA; se faltar algum, solte e toque novamente apenas esse canal. Aguardando deteccao dos quatro canais (ate 60 s)...", flush=True)
        physical = {}
        physical_times = {}
        deadline = time.monotonic()+60
        while time.monotonic() < deadline and len(physical) < 4:
            result = session.command("TOUCHGET")
            physical={channel: value["latency_ms"] for channel,value in result.items()}
            physical_times={channel: value["event_us"] for channel,value in result.items()}
            print(f"Canais confirmados pela ESP: {sorted(physical)}", flush=True)
            until=time.monotonic()+1
            while time.monotonic()<until: session.receive(.1)
        session.command("TOUCHOFF")
        session.command("STOP")
        if len(physical) != 4:
            raise RuntimeError(f"Teste fisico incompleto; canais detectados: {sorted(physical)}")
        (destination/"physical_touch.json").write_text(json.dumps({"run": physical_run,
            "latency_ms_by_channel": physical,
            "detection_span_ms": (max(physical_times.values())-min(physical_times.values()))/1000, "definition": "callback de deteccao ate inicio da tarefa"}, indent=2))
        print(f"Latencias fisicas (ms): {physical}. Proximas cinco repeticoes sao automaticas.", flush=True)
        for repetition in range(1, 6):
            print(f"\nRepeticao {repetition}/5. Solte todos os touches.", flush=True)
            session.receive(1)
            first = len(session.records)
            run = None
            run = session.command("START")
            if run is None:
                raise RuntimeError("Firmware antigo: START nao retornou RUN")
            started = time.monotonic()
            lateness=0.0
            for at, kind, action in SCENARIO:
                while time.monotonic()-started < at:
                    session.receive(min(.2, at-(time.monotonic()-started)))
                lateness=max(lateness,time.monotonic()-started-at)
                if kind == "command":
                    session.command(action)
                else:
                    print(f"[{at:.0f}s] APERTE E SOLTE AGORA: {action} (janela de 6 s)", flush=True)
                    session.save("instruction", action.encode(), run=run, scenario_s=at)
            while time.monotonic()-started < 60:
                session.receive(.2)
            measurement_end=time.monotonic()
            session.command("STOP")
            deadline = time.monotonic()+10
            while time.monotonic() < deadline:
                session.receive(.2)
                records = session.records[first:]
                finals = {}
                event_ids = set()
                for outer in records:
                    record = decode(outer)
                    if record.get("run") != run:
                        continue
                    if record.get("kind") == "metric" and record.get("final"):
                        finals[record["task"]] = record
                    elif record.get("kind") == "event":
                        event_ids.add((record["task"], record["event_id"]))
                if set(finals) == TASKS and len(event_ids) >= sum(r.get("events_expected", r["runs"]) for r in finals.values()):
                    break
            records = session.records[first:]
            report = summarize_run(records, run, args.configuration, args.protocol)
            report.update(repetition=repetition, duration_s=measurement_end-started, planned_duration_s=60,
                          schedule_lateness_max_s=lateness, physical_touch_shared=True,
                          physical_touch_ms_by_channel=physical, touch_latency_max_ms=max(physical.values()))
            path = destination / f"repetition_{repetition}.json"
            path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
            print(f"Salvo {path}: {report['conclusion']}", flush=True)
            write_tables(args.output)
    except (Exception, KeyboardInterrupt) as error:
        failure={"status":"interrupted", "error":str(error), "repetition":repetition, "run":run,
                 "configuration":args.configuration,"protocol":args.protocol,
                 "physical_touch_partial":locals().get("physical",{})}
        (destination/"failure.json").write_text(json.dumps(failure,indent=2,ensure_ascii=False))
        if repetition and session is not None:
            partial=summarize_run(session.records[first:],run,args.configuration,args.protocol)
            partial.update(repetition=repetition,conclusion="INCOMPLETO",interrupted=True,error=str(error),
                           physical_touch_shared=True,physical_touch_ms_by_channel=physical,
                           touch_latency_max_ms=max(physical.values(),default=None))
            (destination/f"repetition_{repetition}.json").write_text(json.dumps(partial,indent=2,ensure_ascii=False))
            write_tables(args.output)
        raise
    finally:
        if session is not None: session.close()


if __name__ == "__main__":
    main()
