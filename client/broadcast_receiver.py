import socket
from time import sleep

from loguru import logger

import connector


def broadcast():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        try:
            s.bind(('0.0.0.0', 31255))
            logger.info("Started broadcast receiver")
            break
        except OSError:
            logger.error("Broadcasting port is busy!")
            sleep(5)
            continue
    magic_bytes = bytes.fromhex("beef")
    server_name = "CopterShow"
    while True:
        data = s.recvfrom(1024)
        # print("received")
        is_broadcast = data[0][0:2] == magic_bytes
        if is_broadcast:
            name = data[0][4:].decode("utf-8")
            if name == server_name:
                port = int.from_bytes(data[0][2:4], byteorder="big")
                address = data[1][0]
                if connector.client is None:
                    logger.info(f"Connecting to {address}:{port}")
                    connector.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    connector.client.connect((address, port))
