"""WinUSB transport for REF's native interface on Windows.

The original PyUSB transport opens the composite parent device. Modern Windows
binds WinUSB to the REF child interface instead, so libusb cannot open it.
"""

import ctypes
from ctypes import wintypes
import time

import fibre.protocol
from fibre.utils import TimeoutError


ERROR_NO_MORE_ITEMS = 259
ERROR_SEM_TIMEOUT = 121
ERROR_OPERATION_ABORTED = 995

DIGCF_PRESENT = 0x00000002
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING = 3
FILE_FLAG_OVERLAPPED = 0x40000000

USBD_PIPE_TYPE_BULK = 2
PIPE_TRANSFER_TIMEOUT = 3


class GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", ctypes.c_ubyte * 8),
    ]


def _guid(data1, data2, data3, *data4):
    return GUID(data1, data2, data3, (ctypes.c_ubyte * 8)(*data4))


USB_DEVICE_CLASS_GUID = _guid(
    0x88BAE032, 0x5A81, 0x49F0, 0xBC, 0x3D, 0xA4, 0xFF, 0x13, 0x82, 0x16, 0xD6
)
DEVICE_PDO_NAME_KEY = (
    _guid(0xA45C254E, 0xDF1C, 0x4EFD, 0x80, 0x20, 0x67, 0xD1, 0x46, 0xA8, 0x50, 0xE0),
    16,
)


class SP_DEVINFO_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("ClassGuid", GUID),
        ("DevInst", wintypes.DWORD),
        ("Reserved", ctypes.c_size_t),
    ]


class DEVPROPKEY(ctypes.Structure):
    _fields_ = [("fmtid", GUID), ("pid", wintypes.DWORD)]


class USB_INTERFACE_DESCRIPTOR(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_ubyte),
        ("bDescriptorType", ctypes.c_ubyte),
        ("bInterfaceNumber", ctypes.c_ubyte),
        ("bAlternateSetting", ctypes.c_ubyte),
        ("bNumEndpoints", ctypes.c_ubyte),
        ("bInterfaceClass", ctypes.c_ubyte),
        ("bInterfaceSubClass", ctypes.c_ubyte),
        ("bInterfaceProtocol", ctypes.c_ubyte),
        ("iInterface", ctypes.c_ubyte),
    ]


class WINUSB_PIPE_INFORMATION(ctypes.Structure):
    _fields_ = [
        ("PipeType", ctypes.c_int),
        ("PipeId", ctypes.c_ubyte),
        ("MaximumPacketSize", wintypes.WORD),
        ("Interval", ctypes.c_ubyte),
    ]


setupapi = ctypes.WinDLL("setupapi", use_last_error=True)
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
winusb = ctypes.WinDLL("winusb", use_last_error=True)

setupapi.SetupDiGetClassDevsW.argtypes = [
    ctypes.POINTER(GUID), wintypes.LPCWSTR, wintypes.HWND, wintypes.DWORD
]
setupapi.SetupDiGetClassDevsW.restype = ctypes.c_void_p
setupapi.SetupDiEnumDeviceInfo.argtypes = [
    ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(SP_DEVINFO_DATA)
]
setupapi.SetupDiEnumDeviceInfo.restype = wintypes.BOOL
setupapi.SetupDiGetDeviceInstanceIdW.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(SP_DEVINFO_DATA),
    wintypes.LPWSTR,
    wintypes.DWORD,
    ctypes.POINTER(wintypes.DWORD),
]
setupapi.SetupDiGetDeviceInstanceIdW.restype = wintypes.BOOL
setupapi.SetupDiGetDevicePropertyW.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(SP_DEVINFO_DATA),
    ctypes.POINTER(DEVPROPKEY),
    ctypes.POINTER(wintypes.ULONG),
    ctypes.POINTER(ctypes.c_ubyte),
    wintypes.DWORD,
    ctypes.POINTER(wintypes.DWORD),
    wintypes.DWORD,
]
setupapi.SetupDiGetDevicePropertyW.restype = wintypes.BOOL
setupapi.SetupDiDestroyDeviceInfoList.argtypes = [ctypes.c_void_p]
setupapi.SetupDiDestroyDeviceInfoList.restype = wintypes.BOOL

kernel32.CreateFileW.argtypes = [
    wintypes.LPCWSTR,
    wintypes.DWORD,
    wintypes.DWORD,
    ctypes.c_void_p,
    wintypes.DWORD,
    wintypes.DWORD,
    wintypes.HANDLE,
]
kernel32.CreateFileW.restype = wintypes.HANDLE
kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
kernel32.CloseHandle.restype = wintypes.BOOL

winusb.WinUsb_Initialize.argtypes = [wintypes.HANDLE, ctypes.POINTER(ctypes.c_void_p)]
winusb.WinUsb_Initialize.restype = wintypes.BOOL
winusb.WinUsb_Free.argtypes = [ctypes.c_void_p]
winusb.WinUsb_Free.restype = wintypes.BOOL
winusb.WinUsb_QueryInterfaceSettings.argtypes = [
    ctypes.c_void_p, ctypes.c_ubyte, ctypes.POINTER(USB_INTERFACE_DESCRIPTOR)
]
winusb.WinUsb_QueryInterfaceSettings.restype = wintypes.BOOL
winusb.WinUsb_QueryPipe.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ubyte,
    ctypes.c_ubyte,
    ctypes.POINTER(WINUSB_PIPE_INFORMATION),
]
winusb.WinUsb_QueryPipe.restype = wintypes.BOOL
winusb.WinUsb_SetPipePolicy.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ubyte,
    wintypes.ULONG,
    wintypes.ULONG,
    ctypes.c_void_p,
]
winusb.WinUsb_SetPipePolicy.restype = wintypes.BOOL
winusb.WinUsb_ReadPipe.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ubyte,
    ctypes.POINTER(ctypes.c_ubyte),
    wintypes.ULONG,
    ctypes.POINTER(wintypes.ULONG),
    ctypes.c_void_p,
]
winusb.WinUsb_ReadPipe.restype = wintypes.BOOL
winusb.WinUsb_WritePipe.argtypes = winusb.WinUsb_ReadPipe.argtypes
winusb.WinUsb_WritePipe.restype = wintypes.BOOL


def _iter_ref_native_paths():
    device_info_set = setupapi.SetupDiGetClassDevsW(
        ctypes.byref(USB_DEVICE_CLASS_GUID), None, None, DIGCF_PRESENT
    )
    invalid_handle = ctypes.c_void_p(-1).value
    if device_info_set == invalid_handle:
        return

    try:
        index = 0
        while True:
            device_info = SP_DEVINFO_DATA()
            device_info.cbSize = ctypes.sizeof(device_info)
            if not setupapi.SetupDiEnumDeviceInfo(
                device_info_set, index, ctypes.byref(device_info)
            ):
                if ctypes.get_last_error() == ERROR_NO_MORE_ITEMS:
                    break
                index += 1
                continue
            index += 1

            instance_id = ctypes.create_unicode_buffer(512)
            if not setupapi.SetupDiGetDeviceInstanceIdW(
                device_info_set,
                ctypes.byref(device_info),
                instance_id,
                len(instance_id),
                None,
            ):
                continue
            instance_id_upper = instance_id.value.upper()
            if "VID_1209&PID_0D32&MI_02" not in instance_id_upper:
                continue

            pdo_buffer = ctypes.create_unicode_buffer(512)
            property_type = wintypes.ULONG()
            required_size = wintypes.DWORD()
            property_key = DEVPROPKEY(*DEVICE_PDO_NAME_KEY)
            if not setupapi.SetupDiGetDevicePropertyW(
                device_info_set,
                ctypes.byref(device_info),
                ctypes.byref(property_key),
                ctypes.byref(property_type),
                ctypes.cast(pdo_buffer, ctypes.POINTER(ctypes.c_ubyte)),
                ctypes.sizeof(pdo_buffer),
                ctypes.byref(required_size),
                0,
            ):
                continue
            yield instance_id.value, r"\\?\GLOBALROOT" + pdo_buffer.value
    finally:
        setupapi.SetupDiDestroyDeviceInfoList(device_info_set)


class WinUSBTransport(fibre.protocol.PacketSource, fibre.protocol.PacketSink):
    def __init__(self, instance_id, path, logger):
        self._logger = logger
        self._name = "WinUSB device " + instance_id
        self._path = path
        self._device_handle = None
        self._interface_handle = ctypes.c_void_p()
        self._read_pipe = None
        self._write_pipe = None
        self._read_size = 64

    def init(self):
        self._device_handle = kernel32.CreateFileW(
            self._path,
            GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            None,
            OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED,
            None,
        )
        if self._device_handle == wintypes.HANDLE(-1).value:
            raise ctypes.WinError(ctypes.get_last_error())
        if not winusb.WinUsb_Initialize(
            self._device_handle, ctypes.byref(self._interface_handle)
        ):
            error = ctypes.get_last_error()
            kernel32.CloseHandle(self._device_handle)
            self._device_handle = None
            raise ctypes.WinError(error)

        descriptor = USB_INTERFACE_DESCRIPTOR()
        if not winusb.WinUsb_QueryInterfaceSettings(
            self._interface_handle, 0, ctypes.byref(descriptor)
        ):
            self.deinit()
            raise ctypes.WinError(ctypes.get_last_error())

        for index in range(descriptor.bNumEndpoints):
            pipe = WINUSB_PIPE_INFORMATION()
            if not winusb.WinUsb_QueryPipe(
                self._interface_handle, 0, index, ctypes.byref(pipe)
            ):
                continue
            if pipe.PipeType != USBD_PIPE_TYPE_BULK:
                continue
            if pipe.PipeId & 0x80:
                self._read_pipe = pipe.PipeId
                self._read_size = pipe.MaximumPacketSize
            else:
                self._write_pipe = pipe.PipeId

        if self._read_pipe is None or self._write_pipe is None:
            self.deinit()
            raise RuntimeError("REF WinUSB interface has no bulk endpoint pair")

    def deinit(self):
        if self._interface_handle.value:
            winusb.WinUsb_Free(self._interface_handle)
            self._interface_handle = ctypes.c_void_p()
        if self._device_handle is not None:
            kernel32.CloseHandle(self._device_handle)
            self._device_handle = None

    def _set_timeout(self, pipe_id, timeout_ms):
        value = wintypes.ULONG(timeout_ms)
        winusb.WinUsb_SetPipePolicy(
            self._interface_handle,
            pipe_id,
            PIPE_TRANSFER_TIMEOUT,
            ctypes.sizeof(value),
            ctypes.byref(value),
        )

    def process_packet(self, packet):
        data = bytes(packet)
        buffer = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
        transferred = wintypes.ULONG()
        self._set_timeout(self._write_pipe, 1000)
        if not winusb.WinUsb_WritePipe(
            self._interface_handle,
            self._write_pipe,
            buffer,
            len(buffer),
            ctypes.byref(transferred),
            None,
        ):
            error = ctypes.get_last_error()
            if error in (ERROR_SEM_TIMEOUT, ERROR_OPERATION_ABORTED):
                raise TimeoutError()
            raise fibre.protocol.ChannelBrokenException()
        return transferred.value

    def get_packet(self, deadline):
        timeout_ms = max(int((deadline - time.monotonic()) * 1000), 1)
        self._set_timeout(self._read_pipe, timeout_ms)
        buffer = (ctypes.c_ubyte * self._read_size)()
        transferred = wintypes.ULONG()
        if not winusb.WinUsb_ReadPipe(
            self._interface_handle,
            self._read_pipe,
            buffer,
            len(buffer),
            ctypes.byref(transferred),
            None,
        ):
            error = ctypes.get_last_error()
            if error in (ERROR_SEM_TIMEOUT, ERROR_OPERATION_ABORTED):
                raise TimeoutError()
            raise fibre.protocol.ChannelBrokenException()
        return bytearray(buffer[: transferred.value])


def discover_channels(
    path,
    serial_number,
    callback,
    cancellation_token,
    channel_termination_token,
    logger,
):
    known_devices = set()
    while not cancellation_token.is_set():
        for instance_id, device_path in _iter_ref_native_paths():
            if instance_id in known_devices:
                continue
            try:
                transport = WinUSBTransport(instance_id, device_path, logger)
                transport.init()
                channel = fibre.protocol.Channel(
                    transport._name,
                    transport,
                    transport,
                    channel_termination_token,
                    logger,
                )
                channel.winusb_transport = transport
                channel_termination_token.subscribe(transport.deinit)
            except OSError as error:
                logger.debug("WinUSB device init failed: {}".format(error))
                continue
            known_devices.add(instance_id)
            callback(channel)
        time.sleep(1)
