import subprocess

try:
    import modules.mavros_wrapper as mavros
except ImportError:
    pass
    #TODO: make faker


def reboot_fcu():
    mavros.reboot_fcu()


def reboot_system():
    subprocess.call(['sudo', 'reboot'])


def restart_service():
    subprocess.call(['sudo', 'systemctl', 'restart', 'show_client.service'])

def stop_service():
    subprocess.call(['sudo', 'systemctl', 'stop', 'show_client.service'])


def calibrate_gyro() -> str:
    mavros.calibrate("gyro")
    return mavros.get_calibration_status()


def calibrate_level() -> str:
    mavros.calibrate("level")
    return mavros.get_calibration_status()


def file_trans(destination: str, data: bytes):
    try:
        with open(destination, "wb") as f:
            f.write(data)
        return True, "success"
    except Exception as e:
        return False, str(*(e.args))
