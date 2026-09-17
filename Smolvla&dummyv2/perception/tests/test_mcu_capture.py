"""Trigger checks only: tests never invoke ST-LINK or access hardware."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('mcu_capture', Path(__file__).resolve().parents[1] / 'tools/capture_mcu_fault.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def test_detects_observed_firmware_and_feedback_faults():
    assert capture.fault_reason({'kind': 'result', 'raw_response': "terminate called after throwing an instance of '"}) == 'firmware_termination'
    assert capture.fault_reason({'kind': 'result', 'raw_response': 'ok 2.31 0.05 \x00\x00.\x004'}) == 'nul_in_ascii_reply'
    assert capture.fault_reason({'kind': 'result', 'error': {'code': 'worker_unavailable'}}) == 'worker_unavailable'


def test_does_not_capture_normal_reply_or_pending_transaction():
    assert capture.fault_reason({'kind': 'result', 'raw_response': '15ok\r\n'}) is None
    assert capture.fault_reason({'kind': 'tx_attempt', 'command': '#GETJPOS'}) is None
    assert capture.fault_reason({'kind': 'result', 'error': {'code': 'io_busy_not_sent'}}) is None
