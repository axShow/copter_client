import socket
import threading
import typing

from loguru import logger

from broadcast_receiver import broadcast

client: typing.Union[socket.socket, None] = None
try:
    from zeroconf import ServiceBrowser, ServiceListener, Zeroconf, ZeroconfServiceTypes, DNSCache




    class MyListener(ServiceListener):

        def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            global client
            info = zc.get_service_info(type_, name, timeout=100)
            if len(info.parsed_addresses()) > 0:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.connect((info.parsed_addresses()[0], info.port))
            print(f"Service {name} updated")

        def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            global client
            if client is not None: client.close()
            client = None
            print(f"Service {name} removed")

        def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            global client
            info = zc.get_service_info(type_, name, timeout=100)
            if info is None:
                if client is not None: client.close()
                client = None
            if client is None:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if len(info.parsed_addresses()) > 0:
                    client.connect((info.parsed_addresses()[0], info.port))
            print(f"Service {name} added, service info: {info}")


    zeroconf = Zeroconf()
    listener = MyListener()
    browser = ServiceBrowser(zeroconf, "_cshow._tcp.local.", listener)
    logger.info("Started zeroconf listener")
except ImportError:
    logger.warning("Zeroconf not installed, using broadcast receiver")
    t2: threading.Thread = threading.Thread(target=broadcast, daemon=True)
    t2.start()
# print('\n'.join(ZeroconfServiceTypes.find()))
# def remove():
#     listener.remove_service(zeroconf, "_cshow._tcp.local.", "server._cshow._tcp.local.")
#
# def restart():
#     print("restarting")
#     global zeroconf, listener, browser
#     zeroconf.remove_service_listener(listener)
#     browser = ServiceBrowser(zeroconf, "_cshow._tcp.local.", listener)
#


# client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#
# client.connect(("127.0.0.1", 8002))
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
"""message = 'Hello, world'
print(f'Sending : "{message}"')
s.send(message.encode('utf-8'))
response = s.recv(1024)
print(f'Received: "{response}"')
s.send("!quit".encode('utf-8'))"""
def is_socket_closed(sock: socket.socket) -> bool:
    try:
        # this will try to read bytes without blocking and also without removing them from buffer (peek only)
        data = sock.recv(16, socket.MSG_DONTWAIT | socket.MSG_PEEK)
        if len(data) == 0:
            return True
    except BlockingIOError:
        return False  # socket is open and reading from it would block
    except ConnectionResetError:
        return True  # socket was closed for some other reason
    except Exception as e:
        print("unexpected exception when checking if a socket is closed")
        return False
    return False