import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
import itertools
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "main"))
from run_experiment import SCENARIO, Session, validate, main as run_main
from collect_logs import BATCH_FIELDS, expand_messages
from summarize_experiments import TASKS, summarize_run, write_tables


def wrap(record, epoch=0):
    return {"mensagem": json.dumps(record), "epoch_ms": epoch}


class ExperimentTest(unittest.TestCase):
    def fixture(self):
        records = []
        event_id = 0
        for task in TASKS:
            count = 12 if task == "NAV_PLAN" else 1
            records.append(wrap(dict(kind="metric", run=1, final=True, task=task, runs=count,
                                     misses=0, drops=0, WCRT_us=2000, windows=3 if count == 12 else 0,
                                     violations=0, logs_lost=0)))
            for i in range(count):
                event_id += 1
                source, kind, epoch = 3, 0, 0
                if task == "NAV_PLAN" and i < 2:
                    source, kind, epoch = 0, i+1, 9000 if i == 0 else 17000
                if task == "FS_TASK":
                    source, epoch = 0, 41000
                records.append(wrap(dict(kind="event", run=1, task=task, event_id=event_id,
                                         source=source, type=kind, inicio_us=100, evento_us=0), epoch))
        # Four simulated touch events are additional task executions.
        for channel in (4, 9, 0, 3):
            task = "FS_TASK" if channel == 0 else "NAV_PLAN"
            for outer in records:
                record=json.loads(outer["mensagem"])
                if record.get("kind") == "metric" and record.get("task") == task:
                    record["runs"]+=1; outer["mensagem"]=json.dumps(record)
            event_id+=1
            records.append(wrap(dict(kind="event", run=1, task=task, event_id=event_id, source=4,
                                     touch_channel=channel, type=0, inicio_us=100, evento_us=0)))
        for label, epoch in [("NAV", 8000), ("TEL", 16000), ("FS", 40000)]:
            records.append(dict(direction="instruction", run=1, mensagem=label+": touch", epoch_ms=epoch))
        records.extend(dict(action="NAV", rtt_ms=2) for _ in range(20))
        return records

    def test_complete_then_missing_event_and_miss(self):
        records = self.fixture()
        report = summarize_run(records, 1, "melhor", "UDP")
        self.assertEqual(report["conclusion"], "OK hard; soft atende (9,10)")
        records.append(wrap(dict(kind="metric", run=1, final=True, task="FUS_IMU", runs=1,
                                 misses=1, drops=0, WCRT_us=6000, windows=0, violations=0, logs_lost=0)))
        self.assertEqual(summarize_run(records, 1, "melhor", "UDP")["conclusion"], "Falha hard")
        records.pop(1)
        self.assertEqual(summarize_run(records, 1, "melhor", "UDP")["conclusion"], "INCOMPLETO")

    def test_scenario_and_configuration(self):
        self.assertEqual(sum(kind == "command" for _, kind, _ in SCENARIO), 20)
        self.assertEqual(sum(kind == "touch" for _, kind, _ in SCENARIO), 0)
        args = SimpleNamespace(configuration="pior", protocol="UDP")
        with self.assertRaises(RuntimeError):
            validate(dict(frequency_mhz=80, preemption=1, slicing=1, protocol="UDP"), args)

    def test_udp_retry_preserves_command_and_sequence(self):
        for action in ("HELLO", "TOUCHOFF", "NAV"):
            session = Session.__new__(Session)
            session.args = SimpleNamespace(protocol="UDP")
            session.sock = Mock()
            session.address = ("127.0.0.1", 6010)
            session.seq = 0
            session.save = Mock()
            def receive(timeout):
                if session.sock.sendto.call_count > 1:
                    return [{"mensagem": f"ACK 1 0 {action} 0 OK", "arrival_mono_ns": 0}]
                return []
            session.receive = receive
            with patch("run_experiment.time.monotonic", side_effect=itertools.count(0, .3)):
                session.command(action)
                self.assertGreater(session.sock.sendto.call_count, 1)
                calls=session.sock.sendto.call_args_list
                self.assertEqual(calls[0],calls[1])

    def test_physical_snapshot_does_not_require_event_datagrams(self):
        session=Session.__new__(Session)
        session.args=SimpleNamespace(protocol="UDP")
        session.sock=Mock(); session.address=("127.0.0.1",6010); session.seq=0
        session.save=Mock()
        session.receive=Mock(return_value=[{"mensagem":"ACK 1 0 TOUCHGET 0 OK TOUCH 537 100 200 300 400 1000 1100 1200 1300", "arrival_mono_ns":0}])
        result=session.command("TOUCHGET")
        self.assertEqual(set(result), {0,3,4,9})
        self.assertEqual(result[9]["latency_ms"], .2)

    def test_batch_preserves_individual_events(self):
        row=["NAV_PLAN", 7, 8, 0, 100, 150, 200, 4, 2, 1, 0, 0, 0, 0, 3, 10, 1]
        packet=json.dumps(dict(kind="batch", protocol="UDP", sntp=True, rows=[row,row])).encode()
        events=[json.loads(data) for data in expand_messages(packet)]
        self.assertEqual(len(events),2)
        self.assertEqual(events[0]["inicio_us"]-events[0]["evento_us"],50)
        self.assertEqual(events[0]["kind"],"event")
        self.assertEqual(events[0]["protocol"],"UDP")
        self.assertEqual(expand_messages(b"ACK 1 OK"),[b"ACK 1 OK"])
        with self.assertRaises(ValueError):
            expand_messages(b'{"kind":"batch","rows":[[1]]}')

    def test_summary_mode_keeps_full_execution_counts(self):
        records=self.fixture()
        filtered=[]
        counts={}
        for outer in records:
            try: record=json.loads(outer.get("mensagem", ""))
            except ValueError: record={}
            if record.get("kind") == "event":
                if record["source"] not in (0,1,4): continue
                counts[record["task"]]=counts.get(record["task"],0)+1
            filtered.append(outer)
        for outer in filtered:
            try: record=json.loads(outer.get("mensagem", ""))
            except ValueError: continue
            if record.get("kind") == "metric":
                record["events_expected"]=counts.get(record["task"],0)
                outer["mensagem"]=json.dumps(record)
        report=summarize_run(filtered,1,"melhor","UDP")
        self.assertEqual(report["conclusion"],"OK hard; soft atende (9,10)")
        self.assertGreater(report["hard_runs"]+report["soft_runs"],report["events_received"])

    def test_summaries_only_and_strict_deadlines(self):
        records=[]
        for task in TASKS:
            records.append(wrap(dict(kind="metric",run=1,final=True,task=task,runs=100,
                misses=1 if task=="NAV_PLAN" else 0,drops=0,WCRT_us=21000,windows=91,
                violations=0,events_expected=0,sim_touch_mask=(1 if task=="FS_TASK" else (1<<3)|(1<<4)|(1<<9)),
                frequency_mhz=160,preemption=1,slicing=1,policy="RM")))
        records.extend(dict(action="NAV",rtt_ms=3,retries=0) for _ in range(19))
        records.append(dict(action="FS",rtt_ms=1200,retries=1))
        report=summarize_run(records,1,"melhor","UDP")
        self.assertEqual(report["conclusion"],"Falha soft")
        self.assertFalse(report["all_deadlines_met"])
        self.assertEqual(report["events_received"],0)
        self.assertEqual(report["rtt_without_retry_max_ms"],3)
        self.assertEqual(report["rtt_with_retry_max_ms"],1200)

    def test_tcp_waits_for_delayed_ack_without_resending(self):
        session=Session.__new__(Session)
        session.args=SimpleNamespace(protocol="TCP")
        session.sock=Mock(); session.address=("127.0.0.1",5000); session.seq=0; session.save=Mock()
        calls=0
        def receive(timeout):
            nonlocal calls
            calls+=1
            return [{"mensagem":"ACK 1 0 NAV 0 OK","arrival_mono_ns":0}] if calls==15 else []
        session.receive=receive
        with patch("run_experiment.time.monotonic",side_effect=itertools.count(0,.5)):
            session.command("NAV")
        session.sock.sendall.assert_called_once()
        self.assertEqual(calls,15)

    def test_connection_failure_is_saved(self):
        with tempfile.TemporaryDirectory() as folder:
            argv=["run_experiment.py","--host","127.0.0.1","--protocol","TCP",
                  "--configuration","melhor","--output",folder]
            with patch("sys.argv",argv), patch("run_experiment.Session",side_effect=RuntimeError("connection failed")):
                with self.assertRaises(RuntimeError): run_main()
            failures=list(Path(folder).glob("*/failure.json"))
            self.assertEqual(len(failures),1)
            self.assertEqual(json.loads(failures[0].read_text())["status"],"interrupted")

    def test_five_repetitions_table(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)/"test"
            folder.mkdir()
            report = summarize_run(self.fixture(), 1, "melhor", "UDP")
            for i in range(5):
                (folder/f"repetition_{i+1}.json").write_text(json.dumps(report))
            rows = write_tables(directory)
            self.assertEqual(rows[0]["Repeticoes"], 5)
            self.assertEqual(rows[0]["Hard misses (%)"], 0)
            self.assertEqual(rows[0]["Conclusao"], "OK hard; soft atende (9,10)")
            self.assertTrue((Path(directory)/"tabela.csv").exists())


if __name__ == "__main__":
    unittest.main()
