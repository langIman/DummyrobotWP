"""Known Dummy USB CDC device; lazy open with no initialization writes."""
class SerialReader:
    def __init__(self):
        self.serial = None
        self.identity = None

    def open(self):
        import serial
        from serial.tools import list_ports
        if self.serial is not None and self.serial.is_open:
            return self.serial
        matches = [p for p in list_ports.comports()
                   if (p.vid, p.pid, p.serial_number) == (0x1209, 0x0D32, '347933853335')]
        if len(matches) != 1:
            raise OSError('device_not_found' if not matches else 'ambiguous_devices')
        p = matches[0]
        # Configure once before opening. On Windows changing timeout on an open
        # port reconfigures the driver, including GetCommState/SetCommState.
        # The transaction loop owns its deadline and polls available bytes.
        s = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=.25)
        s.dtr = False
        s.rts = False
        s.port = p.device
        self.serial = s
        s.open()
        self.identity = (p.device, p.serial_number)
        return s

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None
                self.identity = None


def worker(pipe):
    from motion_backend import serve
    serve(pipe)
