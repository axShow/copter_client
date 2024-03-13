import random
import struct
import time
from time import sleep

import connector
from copterData import CopterData
from utils import send_msg, recv_msg
data = CopterData(name="clover1", battery=54.6, controller_state="ok", flight_mode="stable")
while True:
    socket = connector.client
    if socket is not None:
        try:
            message = data.json()
            send_msg(socket, message.encode('utf-16'))
            print(f'Sending : "{message}"')
            response = recv_msg(socket)
            if response is not None:
                print(f'Received: "{response.decode("utf-16")}"')
        except WindowsError: pass
    sleep(1)
connector.client.close()