import asyncio
import random
import time
from signal import SIGINT, SIGTERM, signal

from loguru import logger

logger.add('copterClient.log', level='INFO')
logger.info("Starting client up...")
import threading
from time import sleep
import socket
import funcs
import connector
from copterData import CopterData, Query, Response, Receive, Heartbeat
from utils import send_msg, recv_msg
name = socket.gethostname()
try:
    import rospy
    from clover import srv
    from clover.srv import SetLEDEffect

    rospy.init_node("my_node")
    get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
except ImportError:
    name = "copter-10"
    # name = "AXSHOW-" + str(random.randint(1111, 9999))
    from faker import get_telemetry

last_server_heartbeat = 0
heartbeat_timeout = 10000
#
# def file(name: str, data: str):
#     data_bytes: bytes = data.encode("utf-16")
#     print(f"writing {name} with {data_bytes.__sizeof__()} bytes")
#     with open(name, "wb") as file:
#         file.write(data_bytes)

running_tasks = []
async def receiver():
    logger.info("Started messages receiver thread")
    while True:
        if connector.client is not None:
            try:
                response = recv_msg(connector.client)
                if response is not None:
                    # print(f'Received: "{response.decode("utf-16")}"')
                    act = response.decode("utf-16")
                    logger.info(f"Received {act}")
                    type = Receive.model_validate_json(act).type
                    if type == "Query":
                        query = Query.model_validate_json(act)
                        async def process_in_another_thread():
                            try:
                                result = await funcs.execute_method(query.method_name, query.args)
                            except Exception as e:
                                logger.exception(e.args[0])
                                result = {"result": False, "details": e.args[0]}
                            response = Response(id=query.id, result=result)
                            send_msg(connector.client, response.model_dump_json().encode("utf-16"))
                            logger.debug(f"Responsed {response}")
                            running_tasks.remove(threading.currentThread())
                        new_task = threading.Thread(target=asyncio.run, args=(process_in_another_thread(),))
                        new_task.start()
                        running_tasks.append(new_task)
                    elif type == "Heartbeat":
                        global last_server_heartbeat
                        last_server_heartbeat = Heartbeat.model_validate_json(act).timestamp
            except KeyboardInterrupt:
                # print("ssshutdownsd")
                break
            except ConnectionResetError:
                pass
            except OSError:
                pass
            except Exception as e:
                logger.exception(e.args[0])
        await asyncio.sleep(0.01)


async def sender():
    while True:
        # print(socket)
        global last_server_heartbeat
        if connector.client is not None:
            if time.time_ns() // 1000000 - last_server_heartbeat > heartbeat_timeout and last_server_heartbeat != 0:
                try:
                    connector.client.close() 
                finally:
                    connector.client = None
                    last_server_heartbeat = 0
                continue
            try:
                telem = get_telemetry()
                data = CopterData(
                    name=name,
                    battery=telem.voltage,
                    controller_state=str(telem.connected),
                    flight_mode=telem.mode)
                message = data.model_dump_json()
                send_msg(connector.client, message.encode("utf-16"))
                logger.debug(f"Sended {message}")
                send_msg(connector.client, Heartbeat(timestamp=time.time_ns() // 1000000).model_dump_json().encode("utf-16"))
                
                
            except KeyboardInterrupt:
                logger.info("Shutting down")
                break
            except ConnectionResetError:
                connector.client = None
                logger.warning("Disconnected...")
                # connector.restart()
            except OSError as e:
                print(e.with_traceback())
                logger.warning("Disconnected (force)...")
                pass
            except Exception as e:
                logger.exception(e.args)

            try:
                await asyncio.sleep(0.5)
            except KeyboardInterrupt:
                logger.info("Shutting down")
                break
            # except rospy.service.ServiceException: pass
        else:
            try:
                await asyncio.sleep(2.5)
            except KeyboardInterrupt:
                logger.info("Shutting down")
                break


def raise_graceful_exit(*args):
    # loop.stop()
    logger.warning("Gracefully shutdown")
    # try:
    #     connector.client.close()
    # except AttributeError:
    #     pass
    raise SystemExit()

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
tasks = [loop.create_task(sender()), loop.create_task(receiver())]
gather = asyncio.gather(*tasks)  # Wait for both tasks to finish
for type_sig in [SIGINT, SIGTERM]:
    signal(type_sig, raise_graceful_exit)
try:
    loop.run_until_complete(gather)
except SystemExit:
    pass
finally:
    logger.info("Stopping loop")
    loop.stop()
    try:
        connector.client.close()
    except AttributeError:
        pass
# t: threading.Thread = threading.Thread(target=receiver, daemon=True)
# t.start()
# t2: threading.Thread = threading.Thread(target=sender, daemon=True)
# t2.start()

# __TAURI_INVOKE__("send_action", {addrName: "ADX", query: {id: 12345, method_name: "land", args: {}}})

