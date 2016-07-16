#Desc: uart server that sends bin data back to requester

from threading import Thread, Event
import serial, time

class Server(Thread):
    def __init__(self, port="/dev/tty.usbmodem1411", baudrate=115200, parity="E", file_path=None):
        super(Server, self).__init__()
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = baudrate
        self.serial.parity = parity
        self.file_path = file_path
        self.data = None

    def get_file_data(self):
        with open(self.file_path, "rb") as binary_file:
            self.data = binary_file.read()

    def write(self, data):
        self.serial.write(data)

    def read(self):
        return self.serial.read()

    def get_prompt(self):
        c = self.read()
        print c
        return c

    def run(self):
        self.get_file_data()
        self.serial.open()
        offs = 0
        lng = len(self.data)
        while lng > 256:
            while(self.get_prompt()!="@"):
                pass

            self.write(self.data[offs:offs + 256])
            offs = offs + 256
            lng = lng - 256

        while (self.read() != "@"):
            pass
        self.write(self.data[offs:offs + lng] + ([0xFF] * (256 - lng)))
        time.sleep(0.1)
        self.write('stop!!')



