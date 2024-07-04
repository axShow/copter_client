from client.modules.flight import force_land, takeoff
import rospy
import asyncio

rospy.init_node("flight")
async def test():
    await takeoff()
    rospy.sleep(10)
    await force_land(descend=True)

asyncio.run(test())