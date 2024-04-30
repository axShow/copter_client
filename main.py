import random
import struct
import threading
import time
from time import sleep
import socket
import connector
import funcs
import logging
from broadcast_receiver import broadcast
from copterData import CopterData, Query, Response
from utils import send_msg, recv_msg
logging.basicConfig(level=logging.INFO)

try:
    import rospy
    from clover import srv
    from clover.srv import SetLEDEffect

    rospy.init_node("my_node")
    get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
except ImportError:
    from faker import get_telemetry


def file(name: str, data: str):
    data_bytes: bytes = data.encode("utf-16")
    print(f"writing {name} with {data_bytes.__sizeof__()} bytes")
    with open(name, "wb") as file:
        file.write(data_bytes)


def thread_2():
    while True:
        socket_r = connector.client
        if socket_r is not None:
            try:
                response = recv_msg(socket_r)
                if response is not None:
                    print(f'Received: "{response.decode("utf-16")}"')
                    act = response.decode("utf-16")
                    if act == "ok": continue
                    query = Query.parse_raw(act)
                    result = funcs.proccess(query.method_name, query.args)
                    response = Response(id=query.id, result=result)
                    send_msg(socket_r, response.json().encode("utf-16"))
            except KeyboardInterrupt:
                break
            except ConnectionResetError:
                connector.client = None
            except Exception as e:
                print(e.args)


t: threading.Thread = threading.Thread(target=thread_2, daemon=True)
t.start()
t2: threading.Thread = threading.Thread(target=broadcast, daemon=True)
t2.start()
name = socket.gethostname()
while True:
    socket = connector.client
    # print(socket)
    if socket is not None:
        try:
            telem = get_telemetry()
            data = CopterData(
                name=name,
                battery=telem.voltage,
                controller_state=str(telem.connected),
                flight_mode=telem.mode)
            message = data.model_dump_json()
            send_msg(socket, message.encode("utf-16"))
            print(f'Sending : "{message}"')

        except KeyboardInterrupt:
            break
        except ConnectionResetError:
            connector.client = None
            # connector.restart()
        except OSError:
            pass

        sleep(1)
        # except rospy.service.ServiceException: pass
    else:
        sleep(5)
connector.client.close()
# __TAURI_INVOKE__("send_action", {addrName: "ADX", query: {id: 12345, method_name: "land", args: {}}})
