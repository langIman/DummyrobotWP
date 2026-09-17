"""ASCII motion framing for the firmware's 64-byte command FIFO entries."""
import re
import math


JOINT_REPLY = re.compile(r'^(?:ok|okok)\s+' +
                        r'\s+'.join([r'(-?\d+(?:\.\d+)?)'] * 6) + r'\s*$')


def joint_reply_complete(raw):
    """Only a newline-terminated, six-finite-angle reply ends a GETJPOS read."""
    if not raw.endswith(b'\n'):
        return False
    for line in raw.decode('ascii', errors='replace').splitlines():
        match = JOINT_REPLY.fullmatch(line.strip())
        if match and all(math.isfinite(float(v)) for v in match.groups()):
            return True
    return False


def format_motion(positions, speed):
    # No exponent notation or long float tails. Leave room for newline/NUL in
    # the controller's fixed-size FIFO; never truncate a command on the host.
    command = '>' + ','.join(format(v, '.3f') for v in [*positions, speed])
    if len(command.encode('ascii')) + 2 > 64:
        raise ValueError('motion_command_exceeds_firmware_buffer')
    return command


def motion_acknowledged(result):
    if result.get('sent') is not True or result.get('error'):
        return False
    lines = [line.strip() for line in result.get('raw_response', '').splitlines() if line.strip()]
    # Firmware Push reports free slots 0..15; execution reports ok. They can
    # interleave as 15ok. A bare slot count is NOT an execution acknowledgement.
    allowed = re.compile(r'(?:(?:[0-9]|1[0-5]))?(?:ok)?')
    return bool(lines) and all(allowed.fullmatch(line) for line in lines) and any(
        line.endswith('ok') for line in lines)
