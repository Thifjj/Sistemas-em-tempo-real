"""Analisa logs JSONL do PC: python3 main/analyze_logs.py log_udp.jsonl."""
import argparse
import collections
import json
import math

DEADLINES = {"FUS_IMU": 5000, "CTRL_ATT": 5000, "NAV_PLAN": 20000,
             "FS_TASK": 10000, "MONITOR_TASK": 500000, "TIME_TASK": 1000000}


def analyze(path):
    groups = collections.defaultdict(list)
    seen = set()
    rtts = []
    lost = invalid = 0
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            try:
                outer = json.loads(line)
                if "rtt_ms" in outer:
                    rtts.append(outer["rtt_ms"])
                record = json.loads(outer["mensagem"]) if "mensagem" in outer else outer
                if record.get("task") not in DEADLINES:
                    continue
                key = (record["task"], record.get("event_id", record["seq"]))
                if key in seen:
                    continue
                seen.add(key)
                groups[record["task"]].append(record)
                lost = max(lost, record.get("perdidos", 0))
            except (ValueError, KeyError, TypeError):
                invalid += 1
    result = {}
    for task, records in groups.items():
        records.sort(key=lambda r: r["inicio_us"])
        responses = [max(0, r["fim_us"]-r["evento_us"]) for r in records]
        executions = [r["fim_us"]-r["inicio_us"] for r in records]
        latencies = [max(0, r["inicio_us"]-r["evento_us"]) for r in records]
        ontime = [r <= DEADLINES[task] for r in responses]
        ordered = sorted(responses)
        result[task] = {"amostras_recebidas": len(records), "misses": ontime.count(False),
                        "WCRT_us": max(responses), "C_wall_max_us": max(executions),
                        "HWM100_us": max(responses),
                        "HWM99_us": ordered[math.ceil(len(ordered)*.99)-1],
                        "latency_max_us": max(latencies),
                        "start_jitter_max_us": max(abs(r["inicio_us"]-r["evento_us"]) for r in records)}
        if task == "NAV_PLAN":
            windows = [ontime[i:i+10] for i in range(len(ontime)-9)]
            result[task]["mk_9_10_janelas_recebidas"] = len(windows)
            result[task]["mk_9_10_violacoes_recebidas"] = sum(sum(w)<9 for w in windows)
    return {"tarefas": result, "rtt_ms": {"amostras": len(rtts), "max": max(rtts, default=None)},
            "logs_perdidos_esp": lost, "linhas_nao_temporais_ou_invalidas": invalid,
            "nota": "Resultados sobre registros recebidos; perdas tornam percentis e janelas incompletos. "
                    "O (m,k)-firm cumulativo do firmware inclui todas as execucoes processadas. "
                    "Use um arquivo por boot/cenario; duracao de parede inclui preempcoes."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("arquivo")
    args = parser.parse_args()
    print(json.dumps(analyze(args.arquivo), indent=2, ensure_ascii=False))
