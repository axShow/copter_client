import asyncio
import threading
import time
from modules.led import set_effect
from modules import other
from modules.animation_parser import Animation
from modules.flight import force_land, navto, kill_switch, land
from loguru import logger
INTERRUPTER = threading.Event()


async def run_animation(start_ts, offset):
    system_ts = lambda: time.time_ns() // 1000000
    anim = Animation("animations/animation.axsanim")
    anim.load()
    if anim.config.reboot_fcu:
        other.reboot_fcu()
    print("delta", start_ts - system_ts())
    print("offset", offset)
    delay = start_ts - (system_ts()+offset*2)
    if delay < 0:
        delay = 0
    print("itog", delay)
    await asyncio.sleep(delay/1000)
    for frame in anim.frames:
        if INTERRUPTER.is_set():
            logger.warning
            break

        if frame.action == "land":
            print("landing")
            await land(descend=True, interrupter=INTERRUPTER)

        elif frame.action == "force_land":
            print("force landing")
            await force_land(descend=True, interrupter=INTERRUPTER)

        elif frame.action == "disarm":
            print("disarming")
            kill_switch()

        else: 
            navto(x=frame.x, y=frame.y, z=frame.z, 
                yaw=frame.yaw,
                frame_id=frame.nav_frame, 
                auto_arm=frame.action == "arm", 
                speed=1)
            set_effect(effect="fade", r=frame.r, g=frame.g, b=frame.b)
            
        # print(frame.action, frame.x, frame.y, frame.z, frame.nav_frame)
        await asyncio.sleep(frame.delay)

