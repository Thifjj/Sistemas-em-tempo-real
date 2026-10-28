"""Build comparison tables: python3 main/summarize_experiments.py resultados."""
import argparse
import collections
import csv
import json
from pathlib import Path

TASKS = {"FUS_IMU", "CTRL_ATT", "NAV_PLAN", "FS_TASK", "MONITOR_TASK", "TIME_TASK"}


def decode(outer):
    try:
        return json.loads(outer.get("mensagem", ""))
    except (ValueError, TypeError):
        return {}


def summarize_run(records, run, configuration, protocol):
    finals, events, cpu = {}, [], {}
    cpu_samples=collections.defaultdict(list)
    network_delays=[]
    seen = set()
    for outer in records:
        record = decode(outer)
        if record.get("run") != run:
            continue
        if record.get("kind") == "metric" and record.get("final"):
            finals[record["task"]] = record
        elif record.get("kind") == "cpu":
            cpu[record["core"]] = record["busy_pct"]
            cpu_samples[record["core"]].append((record["busy_pct"],record.get("interval_us",1)))
        elif record.get("kind") == "event":
            key = (record["task"], record["event_id"])
            if key not in seen:
                seen.add(key)
                events.append((outer, record))
    hard = [r for task, r in finals.items() if task != "NAV_PLAN"]
    soft = finals.get("NAV_PLAN", {})
    hard_runs = sum(r["runs"] for r in hard)
    hard_misses = sum(r["misses"] for r in hard)
    touch = [(r["inicio_us"]-r["evento_us"])/1000 for _, r in events if r["source"] == 0]
    activation = []
    for outer, r in events:
        if (r.get("source") in (1,4) and r.get("pc_send_ms") and r.get("sntp")
                and outer.get("pc_clock_synchronized")):
            activation.append(r["received_epoch_ms"] + (r["inicio_us"]-r["received_us"])/1000-r["pc_send_ms"])
    rtts = [r["rtt_ms"] for r in records if "rtt_ms" in r and r.get("action") in ("NAV", "FS", "T4", "T9", "T0", "T3")]
    for outer in records:
        record=decode(outer)
        if record.get("run") == run and record.get("sntp") and outer.get("pc_clock_synchronized") and record.get("send_epoch_ms"):
            network_delays.append(outer["epoch_ms"]-record["send_epoch_ms"])
    measurements=[r for r in records if "rtt_ms" in r and r.get("action") in ("NAV","FS","T4","T9","T0","T3")]
    clean_rtt=[r["rtt_ms"] for r in measurements if not r.get("retries",0)]
    retry_rtt=[r["rtt_ms"] for r in measurements if r.get("retries",0)]
    simulated_channels = {r.get("touch_channel") for _, r in events if r.get("source") == 4}
    for r in finals.values():
        simulated_channels.update(channel for channel in (4,9,0,3) if r.get("sim_touch_mask",0) & (1 << channel))
    pc_clock_confirmed=any(r.get("pc_clock_synchronized") for r in records)
    if pc_clock_confirmed:
        activation.extend(r["pc_activation_max_us"]/1000 for r in finals.values() if r.get("sntp") and r.get("pc_samples"))
    touch_checks = [{"touch": channel, "observed_in_window": channel in simulated_channels}
                    for channel in (4, 9, 0, 3)]
    missing = sorted(TASKS-set(finals))
    inactive = sorted(t for t, r in finals.items() if not r["runs"])
    dropped = sum(r.get("drops", 0) for r in finals.values())
    logs_lost = max((r.get("logs_lost", 0) for r in finals.values()), default=0)
    incomplete = (bool(missing or inactive) or len(touch_checks) != 4
                  or not all(t["observed_in_window"] for t in touch_checks) or len(rtts) != 20
                  or soft.get("windows", 0) == 0 or len(events) != sum(r.get("events_expected", r["runs"]) for r in finals.values())
                  or logs_lost > 0)
    conclusion = "INCOMPLETO" if incomplete else (
        "Falha hard" if hard_misses or dropped else
        "Falha soft" if soft.get("misses",0) else
        "Soft fora de (9,10)" if soft.get("violations", 0) else "OK hard; soft atende (9,10)")
    wcrt = max((r["WCRT_us"] for r in finals.values()), default=0)/1000
    reasons=[]
    if missing: reasons.append("Resumos finais ausentes: "+", ".join(missing))
    if inactive: reasons.append("Tarefas sem execucao: "+", ".join(inactive))
    if logs_lost: reasons.append(f"Eventos descartados na ESP: {logs_lost}")
    expected_events=sum(r.get("events_expected",r["runs"]) for r in finals.values())
    if len(events)!=expected_events: reasons.append(f"Eventos recebidos/esperados: {len(events)}/{expected_events}")
    if soft.get("windows",0)==0: reasons.append("Sem janela completa (9,10)")
    if len(rtts)!=20: reasons.append(f"Comandos confirmados: {len(rtts)}/20")
    if not all(check["observed_in_window"] for check in touch_checks): reasons.append("Touches simulados incompletos")
    metadata=next(iter(finals.values()),{})
    return {"incomplete_reasons":reasons,
            "frequency_mhz":metadata.get("frequency_mhz"), "preemption":metadata.get("preemption"),
            "slicing":metadata.get("slicing"),"policy":metadata.get("policy"),
            "all_deadlines_met":not incomplete and not hard_misses and not soft.get("misses",0) and not dropped,
            "cpu_mean_received_pct":{core:sum(value*weight for value,weight in samples)/sum(weight for _,weight in samples)
                for core,samples in cpu_samples.items() if sum(weight for _,weight in samples)},
            "cpu_max_received_pct":{core:max(value for value,_ in samples) for core,samples in cpu_samples.items()},
            "rtt_without_retry_max_ms":max(clean_rtt,default=None), "rtt_with_retry_max_ms":max(retry_rtt,default=None),
            "retransmissions":sum(r.get("retries",0) for r in measurements),
            "esp_to_pc_max_ms":max(network_delays,default=None),
            "clock_comparison_suspect":any(value<0 for value in network_delays+activation),
            "resource_analysis":{task:{"contentions":r.get("lock_samples",0),
                "wait_max_us":r.get("lock_wait_max_us",0),"wait_total_us":r.get("lock_wait_total_us",0),
                "lower_priority_holder_observed":r.get("lower_priority_holder",0),
                "off_cpu_us":r.get("off_cpu_total_us",0),"switches":r.get("switches_total",0)} for task,r in finals.items()},
            "resource_note":"Mutex drone com heranca de prioridade; espera inclui custo da chamada. Off-CPU inclui bloqueios e desescalonamento, nao identifica sozinho a tarefa causadora.",
            "theme": "Drone", "configuration": configuration, "protocol": protocol, "run": run,
            "wcrt_ms": wcrt, "hwm100_ms": wcrt,
            "mk": "(9,10)", "mk_windows": soft.get("windows", 0), "mk_violations": soft.get("violations", 0),
            "hard_runs": hard_runs, "hard_misses": hard_misses,
            "soft_runs": soft.get("runs", 0), "soft_misses": soft.get("misses", 0),
            "touch_latency_max_ms": max(touch, default=None),
            "protocol_rtt_max_ms": max(rtts, default=None), "rtt_samples": len(rtts),
            "pc_to_task_max_ms": max(activation, default=None),
            "touch_checks": touch_checks, "missing_tasks": missing, "inactive_tasks": inactive,
            "events_received": len(events), "logs_lost": logs_lost, "task_drops": dropped,
            "cpu_latest_pct": cpu, "tasks": finals, "conclusion": conclusion}


def write_tables(directory):
    directory = Path(directory)
    groups = collections.defaultdict(list)
    for path in directory.glob("*/repetition_*.json"):
        report = json.loads(path.read_text(encoding="utf-8"))
        groups[report["configuration"], report["protocol"]].append(report)
    rows = []
    for (configuration, protocol), reports in sorted(groups.items()):
        hard_runs = sum(r["hard_runs"] for r in reports)
        soft_runs = sum(r["soft_runs"] for r in reports)
        conclusions = {r["conclusion"] for r in reports}
        complete = len(reports) == 5 and "INCOMPLETO" not in conclusions
        def maximum(field):
            values = [r[field] for r in reports if r.get(field) is not None]
            return round(max(values), 3) if values else "N/D"
        rows.append({"Tema": "Drone", "Configuracao": configuration, "Protocolo": protocol,
                     "MHz":reports[0].get("frequency_mhz"),"Preempcao":reports[0].get("preemption"),
                     "Slicing":reports[0].get("slicing"),"Politica":reports[0].get("policy"),
                     "Todos deadlines atendidos":complete and all(r.get("all_deadlines_met",False) for r in reports),
                     "Repeticoes": len(reports), "WCRT (ms)": maximum("wcrt_ms"),
                     "HWM100 (ms)": maximum("hwm100_ms"),
                     "(m,k)-firm": f"(9,10); violacoes={sum(r['mk_violations'] for r in reports)}; janelas={sum(r['mk_windows'] for r in reports)}",
                     "Hard misses (%)": round(100*sum(r["hard_misses"] for r in reports)/hard_runs, 4) if hard_runs else "N/D",
                     "Soft misses (%)": round(100*sum(r["soft_misses"] for r in reports)/soft_runs, 4) if soft_runs else "N/D",
                     "Latencia Touch max (ms)": maximum("touch_latency_max_ms"),
                     "Latencia Protocolo (RTT max, ms)": maximum("protocol_rtt_max_ms"),
                     "RTT sem reenvio max (ms)":maximum("rtt_without_retry_max_ms"),
                     "RTT com reenvio max (ms)":maximum("rtt_with_retry_max_ms"),
                     "Reenvios":sum(r.get("retransmissions",0) for r in reports),
                     "ESP ate PC max (ms)":maximum("esp_to_pc_max_ms"),
                     "PC ate task max (ms)": maximum("pc_to_task_max_ms"),
                     "CPU max recebida (%)":max((max(r.get("cpu_max_received_pct",{}).values(),default=0) for r in reports),default=0),
                     "Conclusao": "; ".join(sorted(conclusions)) if complete else "INCOMPLETO: verificar repeticoes e cobertura"})
    if rows:
        with (directory/"tabela.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        fields = list(rows[0])
        lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
        lines += ["| " + " | ".join(str(row[f]) for f in fields) + " |" for row in rows]
        (directory/"tabela.md").write_text("\n".join(lines)+"\n\nLatencia Protocolo = maior RTT medido entre envio do comando e recebimento do ACK no PC; inclui processamento, filas e eventuais reenvios. Nao representa atraso de mao unica.\n", encoding="utf-8")
    return rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    print(json.dumps(write_tables(parser.parse_args().directory), indent=2, ensure_ascii=False))
