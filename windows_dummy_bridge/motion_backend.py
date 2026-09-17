"""V4 serial transactions; only the supplied payload is ever written."""
import time
from motion_protocol import motion_acknowledged, joint_reply_complete

MAX_RESPONSE_BYTES = 65536
READ_POLL_SECONDS = .001
SERIAL_READ_STRATEGY = 'nonblocking_poll_v1'


def exchange(reader, payload, read_timeout_ms):
    started = time.monotonic()
    result = dict(sent=False, bytes_written=0, write_status='not_attempted',
                  raw_response='', raw_response_hex='', pending_response='',
                  pending_response_hex='', read_timed_out=False)
    received = bytearray()
    stage = 'open'
    try:
        s = reader.open()
        result.update(port=reader.identity[0], device_id=reader.identity[1])
        # Preserve already-buffered replies separately; never silently flush.
        stage = 'pending_read'
        pending_count = s.in_waiting
        if pending_count > MAX_RESPONSE_BYTES and payload:
            result['error'] = dict(code='pending_buffer_full', message='Read pending bytes with an empty command first')
            return result
        if pending_count:
            pending = s.read(min(pending_count, MAX_RESPONSE_BYTES))
            result.update(pending_response=pending.decode('latin-1'), pending_response_hex=pending.hex())
        stage = 'write'
        if payload:
            result.update(sent=None, bytes_written=None, write_status='unknown')
            count = s.write(payload)  # One attempt, never retry a partial write.
            result.update(sent=count == len(payload), bytes_written=count,
                          write_status='complete' if count == len(payload) else 'partial')
            if count != len(payload):
                result['error'] = dict(code='partial_write', message='Command not fully written; no retry')
                reader.close()
                return result
        else:
            result.update(sent=True, write_status='complete')
        stage = 'read'
        deadline = time.monotonic() + read_timeout_ms / 1000
        while read_timeout_ms and time.monotonic() < deadline:
            remaining = MAX_RESPONSE_BYTES - len(received)
            if not remaining:
                result['response_limit_reached'] = True
                break
            # Never change serial properties here: pySerial reconfigures the
            # Windows driver on every assignment, even if the value is equal.
            available = s.in_waiting
            data = s.read(min(available, remaining)) if available else b''
            if not data:
                pause = min(READ_POLL_SECONDS, max(0, deadline - time.monotonic()))
                if pause:
                    time.sleep(pause)
                continue
            received.extend(data)
            # Finish a structured motion transaction on a complete, validated
            # acknowledgement, not an unconditional half-second delay.
            if (payload.startswith(b'>') and received.endswith(b'\n')
                    and motion_acknowledged({'sent':result['sent'],
                                             'raw_response':received.decode('latin-1')})):
                result['read_ended_on_ack'] = True
                break
            if payload.strip() == b'#GETJPOS' and joint_reply_complete(received):
                result['read_ended_on_joint_reply'] = True
                break
        result['read_timed_out'] = bool(read_timeout_ms and not received)
        result['read_wait_skipped'] = read_timeout_ms == 0
        result['reply_correlation'] = 'unavailable_in_ascii_protocol'
    except Exception as exc:
        result['error'] = dict(code=stage + '_error', message=type(exc).__name__ + ': ' + str(exc))
        reader.close()
    finally:
        result.update(raw_response=received.decode('latin-1'), raw_response_hex=received.hex(),
                      serial_elapsed_ms=round((time.monotonic() - started) * 1000, 3),
                      received_at_ms=time.time_ns() // 1_000_000)
    return result


def serve(pipe):
    from device import SerialReader
    reader = SerialReader()
    try:
        while True:
            message = pipe.recv()
            if message == 'close':
                break
            result = exchange(reader, message['payload'], message['read_timeout_ms'])
            pipe.send(result)
    except (EOFError, BrokenPipeError):
        pass
    finally:
        reader.close()  # No STOP or DISABLE on close.
        pipe.close()
