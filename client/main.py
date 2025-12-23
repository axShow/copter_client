import asyncio
import random
import time
import uvicorn
from signal import SIGINT, SIGTERM, signal
from modules.led import get_color
from loguru import logger

from modules.mavros_wrapper import get_sys_status

logger.add('copterClient.log', level='INFO')
logger.info("Starting client up...")
import threading
from time import sleep
import socket
import funcs
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# import connector
from copterData import CopterData
from utils import check_controller_state, send_msg, recv_msg
name = socket.gethostname()
try:
    import rospy
    from clover import srv
    from clover.srv import SetLEDEffect

    rospy.init_node("axshow")
    get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
except ImportError:
    name = "AXSHOW-" + str(random.randint(1111, 9999))
    from faker import get_telemetry


running_tasks = []
app = FastAPI()
app.include_router(funcs.router)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

async def sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    while True:
        try:
            telem = get_telemetry()
            status, state = check_controller_state(telem.z, telem.voltage)
            data = CopterData(
                name=name,
                battery=round(telem.voltage, 2),
                x=round(telem.x, 2),
                y=round(telem.y, 2),
                z=round(telem.z, 2),
                state=state,
                status=status,
                flight_mode=telem.mode,
                color=get_color())
            message = data.model_dump_json()
            send_msg(sock, message.encode("utf-16"))
            logger.debug(f"Sended {message}")
            
            
        except KeyboardInterrupt:
            logger.info("Shutting down")
            break
        except Exception as e:
            logger.exception(e.args)

        try:
            await asyncio.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("Shutting down")
            break


def raise_graceful_exit(*args):
    logger.warning("Gracefully shutdown")
    raise SystemExit()

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
config = uvicorn.Config(app, host="0.0.0.0", port=8034, log_level="warning")
server = uvicorn.Server(config)
logger.info("Starting http server on 0.0.0.0:8034...")
tasks = [loop.create_task(sender()), server.serve()]
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