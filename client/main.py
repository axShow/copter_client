from loguru import logger
logger.add('copterClient.log', level='INFO')
logger.info("Starting client up...")
import threading
from time import sleep
import socket
import funcs
import connector
from copterData import CopterData, Query, Response
from utils import send_msg, recv_msg
try:
    import rospy
    from clover import srv
    from clover.srv import SetLEDEffect

    rospy.init_node("my_node")
    get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
except ImportError:
    from faker import get_telemetry

#
# def file(name: str, data: str):
#     data_bytes: bytes = data.encode("utf-16")
#     print(f"writing {name} with {data_bytes.__sizeof__()} bytes")
#     with open(name, "wb") as file:
#         file.write(data_bytes)


def receiver():
    logger.info("Started messages receiver thread")
    while True:
        if connector.client is not None:
            try:
                response = recv_msg(connector.client)
                if response is not None:
                    # print(f'Received: "{response.decode("utf-16")}"')
                    act = response.decode("utf-16")
                    if act == "ok": continue
                    logger.info(f"Received {act}")
                    query = Query.parse_raw(act)
                    result = funcs.proccess(query.method_name, query.args)
                    response = Response(id=query.id, result=result)
                    send_msg(connector.client, response.json().encode("utf-16"))
                    logger.debug(f"Responsed {response}")
            except KeyboardInterrupt:
                break
            except ConnectionResetError:
                pass
            except OSError:
                pass
            except Exception as e:
                logger.exception(e.args[0])


t: threading.Thread = threading.Thread(target=receiver, daemon=True)
t.start()
name = socket.gethostname()
while True:
    # print(socket)
    if connector.client is not None:
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

        except KeyboardInterrupt:
            logger.info("Shutting down")
            break
        except ConnectionResetError:
            connector.client = None
            logger.warning("Disconnected...")
            # connector.restart()
        except OSError:
            logger.warning("Disconnected...")
            pass
        except Exception as e:
            logger.exception(e.args[0])

        try:
            sleep(1)
        except KeyboardInterrupt:
            logger.info("Shutting down")
            break
        # except rospy.service.ServiceException: pass
    else:
        try:
            sleep(5)
        except KeyboardInterrupt:
            logger.info("Shutting down")
            break
try:
    connector.client.close()
except AttributeError:
    pass
# __TAURI_INVOKE__("send_action", {addrName: "ADX", query: {id: 12345, method_name: "land", args: {}}})
