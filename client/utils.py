import select
import struct
from typing_extensions import Tuple

from copterData import DroneState
from modules import flight
from modules.mavros_wrapper import get_sys_status


DRONE_CRITICAL_BATTERY_VOLTAGE = 14.5
def send_msg(sock, msg: bytes):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    sock.sendto(msg, ("255.255.255.255", 9009))


def recv_msg(sock):
    read = select.select([sock], [], [], 0.01)[0]
    # Read message length and unpack it into an integer
    if len(read) == 0: return None
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(sock, msglen)


def recvall(sock, n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def check_controller_state(z, battery) -> Tuple[str, DroneState]:
        sys_status = get_sys_status()
        state = DroneState.OPERATIONAL
        if sys_status == "STANDBY":
            sys_status = "GROUND"
        if sys_status == "ACTIVE":
            sys_status = flight.last_action
            if sys_status == "IDLE":
                sys_status = str(round(z, 1)) + "m"
                state = DroneState.OPERATIONAL
            else:
                state = DroneState.BUSY
        
        if sys_status == "NO_FCU":
            state = DroneState.EMERGENCY
        elif sys_status in ["CRITICAL", "EMERGENCY", "POWEROFF", "TERMINATION"]:
            state = DroneState.EMERGENCY
        elif sys_status in ["CALIBRATING", "BOOT", "UNINIT"]:
            state = DroneState.BUSY
            
        if battery <= DRONE_CRITICAL_BATTERY_VOLTAGE and state != DroneState.EMERGENCY:
            state = DroneState.WARN
            sys_status = "LOW BATTERY"
        return sys_status, state