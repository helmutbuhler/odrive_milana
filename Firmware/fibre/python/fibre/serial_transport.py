"""
Provides classes that implement the StreamSource/StreamSink and
PacketSource/PacketSink interfaces for serial ports.
"""

import os
import re
import time
import traceback
import serial
import serial.tools.list_ports
import fibre
from fibre.utils import TimeoutError

DEFAULT_BAUDRATE = 115200
DEFAULT_STOP_BITS = 1

class SerialStreamTransport(fibre.protocol.StreamSource, fibre.protocol.StreamSink):
    def __init__(self, port, baud, stop_bits):
        self._timeout = 1
        sb = None
        if stop_bits == 1:
            sb = serial.STOPBITS_ONE
        elif stop_bits == 2:
            sb = serial.STOPBITS_TWO
        self._dev = serial.Serial(port, baud, timeout=self._timeout,
                                  stopbits=sb)

    def process_bytes(self, bytes):
        self._dev.write(bytes)

    def get_bytes(self, n_bytes, deadline):
        """
        Returns n bytes unless the deadline is reached, in which case the bytes
        that were read up to that point are returned. If deadline is None the
        function blocks forever. A deadline before the current time corresponds
        to non-blocking mode.
        """
        # Only set new timeout value if it is reasonably different from the old one (e.g. 20% as below)
        # Otherwise it adds significant overhead (at least under Win10) as the port is reset with every reconfiguration
        if deadline is None and self._timeout is not None:
            self._timeout = None
            self._dev.timeout = None
        elif deadline is not None:
            new_timeout = max(deadline - time.monotonic(), 0)
            if abs(new_timeout - self._timeout) > self._timeout * 0.2:
                self._timeout = new_timeout
                self._dev.timeout = new_timeout
        return self._dev.read(n_bytes)

    def get_bytes_or_fail(self, n_bytes, deadline):
        result = self.get_bytes(n_bytes, deadline)
        if len(result) < n_bytes:
            raise TimeoutError("expected {} bytes but got only {}", n_bytes, len(result))
        return result

    def close(self):
        self._dev.close()


def find_dev_serial_ports():
    try:
        return ['/dev/' + x for x in os.listdir('/dev')]
    except FileNotFoundError:
        return []

def find_pyserial_ports():
    return [x.device for x in serial.tools.list_ports.comports()]


def discover_channels(path, serial_number, callback, cancellation_token, channel_termination_token, logger):
    """
    Scans for serial ports that match the path spec.
    This function blocks until cancellation_token is set.
    Channels spawned by this function run until channel_termination_token is set.
    """
    baud = DEFAULT_BAUDRATE
    stop_bits = DEFAULT_STOP_BITS
    if path == None:
        # This regex should match all desired port names on macOS,
        # Linux and Windows but might match some incorrect port names.
        regex = r'^(/dev/tty\.usbmodem.*|/dev/ttyACM.*|COM[0-9]+)$'
    else:
        path_parts = path.split("_")
        if len(path_parts) == 1:
            path = path_parts[0]
        elif len(path_parts) == 2:
            path = path_parts[0]
            baud = int(path_parts[1])
        elif len(path_parts) == 3:
            baud = int(path_parts[1])
            if path_parts[2] == "8N1":
                stop_bits = 1
            elif path_parts[2] == "8N2":
                stop_bits = 2
            else:
                raise Exception("Unexpected serial path format: {} Last part must be 8N1 or 8N2".format(path))
            path = path_parts[0]
        else:
            raise Exception("Unexpected serial path format: {}".format(path))
        regex = "^" + path + "$"

    known_devices = []
    def device_matcher(port_name):
        if port_name in known_devices:
            return False
        return bool(re.match(regex, port_name))

    def did_disconnect(port_name, device):
        device.close()
        # TODO: yes there is a race condition here in case you wonder.
        known_devices.pop(known_devices.index(port_name))

    while not cancellation_token.is_set():
        all_ports = find_pyserial_ports() + find_dev_serial_ports()
        new_ports = filter(device_matcher, all_ports)
        for port_name in new_ports:
            try:
                serial_device = SerialStreamTransport(port_name, baud, stop_bits)
                input_stream = fibre.protocol.PacketFromStreamConverter(serial_device)
                output_stream = fibre.protocol.StreamBasedPacketSink(serial_device)
                channel = fibre.protocol.Channel(
                        "serial port {}@{}".format(port_name, baud),
                        input_stream, output_stream, channel_termination_token, logger)
                channel.serial_device = serial_device
            except serial.serialutil.SerialException:
                logger.debug("Serial device init failed. Ignoring this port. More info: " + traceback.format_exc())
                known_devices.append(port_name)
            else:
                known_devices.append(port_name)
                channel._channel_broken.subscribe(lambda: did_disconnect(port_name, serial_device))
                callback(channel)
        time.sleep(1)
