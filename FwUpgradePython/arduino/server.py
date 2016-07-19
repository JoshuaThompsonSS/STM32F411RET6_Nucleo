#Desc: uart server that sends bin data back to requester

from threading import Thread, Event
import serial, time

class Server(Thread):
    def __init__(self, port="COM11", baudrate=9600, file_path="blink.bin"):
        super(Server, self).__init__()
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = baudrate
        #self.serial.parity = parity
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
        data_size = 8
        self.get_file_data()
        self.serial.open()
        offs = 0
        lng = len(self.data)
        print "data len %d" % lng
        while lng > data_size:
            while(self.get_prompt()!="@"):
                pass

            self.write(self.data[offs:offs + data_size])
            offs = offs + data_size
            lng = lng - data_size

        while (self.get_prompt() != "@"):
            pass
        self.write(self.data[offs:offs + lng] + (chr(0xFF) * (data_size - lng)))
        time.sleep(0.1)
        self.write('stop!!!!')
		
if __name__ == "__main__":
	from IPython import embed
	embed()



