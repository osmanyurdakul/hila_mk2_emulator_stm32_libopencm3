import serial

class HilaRS485Interface:
    def __init__(self, port) -> None:
        self.hila_rs485_comm = serial.Serial(port, baudrate=56700, timeout=1)
    def hila_transmit(self, command, parameter=None):
        if parameter == None:
            self.hila_rs485_comm.write(("%s\r" %command).encode('ascii'))
        else:
            try:
                self.hila_rs485_comm.write(("%s %s\r" %command).encode('ascii'))
            except:
                print("Parameter is invalid")
    def hila_receive(self):
        response = self.hila_rs485_comm.read_until(b'\r')
        return response.rstrip()
    def hila_disconnect(self):
        self.hila_rs485_comm.close()