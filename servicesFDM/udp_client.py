import socket
import os
import time
import struct
import numpy as np

# aus: https://mineyourlife.net/socket-programmierung-am-beispiel-eines-chat-servers-in-python-2-7/

class UdpClient(object):

    def __init__(self, ip, Port):
        self.ip = ip
        self.port = Port
        # Groesse Empfangspuffer (begrenzt die empfangene Nachricht)
        self.bufsize = 1024


    # schließt die Verbindung zum Server und beendet den Client
    def close(self):
        self.server.close()
        print("(Client)" + "PROGRAMM WIRD GESCHLOSSEN!")
        time.sleep(1)
        os._exit(1)

    # sendet text an den Server
    def send(self, text):
        self.UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket Objekt
        self.UDPSock.setblocking(0)
        self.UDPSock.sendto(text, (str(self.ip), int(self.port)))

    # ließt den buffer auf neue strings vom server, es wird gewartet falls er leer ist
    def recv(self):
        self.UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket Objekt
        self.UDPSock.bind((str(self.ip), int(self.port)))  # verbungsaufbau zum Server
        (data, addr) = self.UDPSock.recvfrom(self.bufsize)
        return data




if __name__ == "__main__":

    port = 5114
    udpClient2 = UdpClientClass('127.0.0.1', port)
    udpClient2.send((struct.pack('fff', 10,1,23)))




