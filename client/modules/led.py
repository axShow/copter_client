import enum
from loguru import logger
from typing_extensions import Tuple
try:
    from led_msgs.msg import LEDStateArray
    import rospy # type: ignore
    from clover.srv import SetLEDEffect # type: ignore

    set_effect_service = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
except ImportError:
    import faker

    set_effect_service = faker.set_effect_service

class LEDEffects(enum.Enum):
    FILL = 'fill'
    BLINK = 'blink'
    BLINK_FAST = 'blink_fast'
    FADE = 'fade'
    WIPE = 'wipe'
    FLASH = 'flash'
    RAINBOW = 'rainbow'
    RAINBOW_FILL = 'rainbow_fill'

def set_effect(*args, **kwargs) -> Tuple[bool, str]:
    try:
        set_effect_service(*args, **kwargs)
        return True, 'success'
    except rospy.ServiceException:
        logger.error("Can't set led effect: service /led/set_effect is unavailable!")
        return False, 'service /led/set_effect is unavailable'
    
def get_color() -> Tuple[int, int, int]:
    try:
        led = rospy.wait_for_message('led/state', LEDStateArray, timeout=0.1).leds[0]
        return led.r, led.g, led.b
    except Exception:
        return faker.data.r, faker.data.g, faker.data.b