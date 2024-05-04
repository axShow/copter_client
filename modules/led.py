from loguru import logger
try:
    import rospy
    from clover.srv import SetLEDEffect

    set_effect_service = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
except ImportError:
    from client import faker

    set_effect_service = faker.set_effect_service

def set_effect(*args, **kwargs) -> tuple[bool, str]:
    try:
        set_effect_service(*args, **kwargs)
        return True, 'success'
    except rospy.ServiceException:
        logger.error("Can't set led effect: service /led/set_effect is unavailable!")
        return False, 'service /led/set_effect is unavailable'