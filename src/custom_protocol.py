import logging
import socket

import struct
import threading

rx_format = '<4L12l12L'
tx_format = '<L4l4L4l'


def separate_packet(packet, num_of_elements):
    index = 0
    for i in num_of_elements:
        yield packet[index:index + i]
        index = i


class PMDCustomProtocol:
    def __init__(self, ip_address, port=18021, offset=0):
        self.offset = offset
        self.address = (ip_address, port)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_rx.bind(('', port))
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tx.setblocking(False)
        self.rx_callback = None

    def start_receive_loop(self, callback):
        self.rx_callback = callback
        threading.Thread(target=self.rx_loop, daemon=True).start()

    def rx_loop(self):
        while True:
            try:
                received, address = self.sock_rx.recvfrom(struct.calcsize(rx_format))
                if address[0] == self.address[0]:
                    unpacked = struct.unpack(rx_format, received)
                    separated = separate_packet(unpacked, (4, 4, 4, 4, 4, 4, 4))
                    packet = {
                        'mode': next(separated),
                        'current': next(separated),
                        'velocity': next(separated),
                        'position': next(separated),
                        'temperature': next(separated),
                        'analog': next(separated),
                        'fault': next(separated),
                    }
                    if self.rx_callback is not None:
                        self.rx_callback(**packet, offset=self.offset)

            except Exception:
                logging.exception('error in the rx loop')

    def send(self, command=0, command_payload=(0, 0, 0, 0), mode=(0, 0, 0, 0), motor_command=(0, 0, 0, 0)):
        packet = struct.pack(tx_format, command, *command_payload, *mode, *motor_command)
        self.sock_tx.sendto(packet, self.address)
