import socket
from zeroconf import ServiceBrowser, ServiceListener, Zeroconf, ZeroconfServiceTypes

client: socket.socket | None = None
class MyListener(ServiceListener):

    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        global client
        info = zc.get_service_info(type_, name)
        if len(info.parsed_addresses()) > 0:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((info.parsed_addresses()[0], info.port))
        print(f"Service {name} updated")

    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        global client
        client.close()
        client = None
        print(f"Service {name} removed")

    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name)
        global client
        if client is None:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if len(info.parsed_addresses()) > 0:
                client.connect((info.parsed_addresses()[0], info.port))
        print(f"Service {name} added, service info: {info}")


zeroconf = Zeroconf()
listener = MyListener()
browser = ServiceBrowser(zeroconf, "_cshow._tcp.local.", listener)

#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
"""message = 'Hello, world'
print(f'Sending : "{message}"')
s.send(message.encode('utf-8'))
response = s.recv(1024)
print(f'Received: "{response}"')
s.send("!quit".encode('utf-8'))"""