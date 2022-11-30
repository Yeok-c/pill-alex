import serial
import time
import numpy as np
# import keyboard

class Pneumatics:
    def __init__(self, 
        suction_read_port='/dev/ttyUSB1', suction_read_baud=115200,
        suction_write_port='/dev/ttyUSB0', suction_write_baud=9600,
        threshold_true_vacuum_kpa = -55.0,
        threshold_not_activated_kpa = -1.0):
        try:
            self.ser_read = serial.Serial(
                suction_read_port, 
                suction_read_baud, 
                timeout=0.01,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff = False,
                rtscts = False,
                dsrdtr=False,
                ) 
            # self.ser_read.open()

            self.ser_write = serial.Serial(
                suction_write_port, 
                suction_write_baud, 
                timeout=0.01,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff = False,
                rtscts = False,
                dsrdtr=False,
                ) 
            # self.ser_read.open()
        except Exception as ex:
            print("Open serial port error " + str(ex))
            exit()
        
        self.threshold_true_vacuum_kpa = threshold_true_vacuum_kpa # -55
        self.threshold_not_activated_kpa = threshold_not_activated_kpa # -1
        

    def get_pressure(self):
        # Returned data 
        # Commands to send, including crc
        # [12][04][00][01][00][01][62][A9]
        # [13][04][00][01][00][01][63][78]
        # [14][04][00][01][00][01][62][CF]
        # [15][04][00][01][00][01][63][1E]

        commands=[
            [0x12, 0x04, 0x00, 0x01, 0x00, 0x01, 0x62, 0xA9],
            [0x13, 0x04, 0x00, 0x01, 0x00, 0x01, 0x63, 0x78],
            [0x14, 0x04, 0x00, 0x01, 0x00, 0x01, 0x62, 0xCF],
            [0x15, 0x04, 0x00, 0x01, 0x00, 0x01, 0x63, 0x1E],
        ]

        pressures = []
        for command in commands:
            response = self.ser_send(self.ser_read, command)
            kpa = int.from_bytes(response[3:5], 'big', signed=True)/10
            pressures.append(kpa)
        return np.array(pressures)
        
    def ser_send(self, port, data):
        if port.isOpen():
            try:
                port.flushInput()  # flush input buffer
                port.flushOutput()  # flush output buffer
                # print('sending:', data)
                port.write(serial.to_bytes(data))
                time.sleep(port.timeout)  # wait 0.5s
                # read 8 byte data
                response = port.read(8)
                # print("read 8 byte data: ", response)

                # self.ser_read.close()
                return response
            except Exception as e1:
                print("communicating error " + str(e1))
        else:
            print("open serial port error")


    def activate(self):
        self.send_data(self.ser_write, '01050000FF008C3A')
        self.send_data(self.ser_write, '01050001FF00DDFA')
        self.send_data(self.ser_write, '01050002FF002DFA')
        self.send_data(self.ser_write, '01050003FF007C3A')
        self.activation_state = True
        print("Activate suction cup")
    
    def deactivate(self):
        self.send_data(self.ser_write, '010500000000CDCA')
        self.send_data(self.ser_write, '0105000100009C0A')
        self.send_data(self.ser_write, '0105000200006C0A')
        self.send_data(self.ser_write, '0105000300003DCA')
        self.activation_state = False
        print("Deactivate suction cup")
    
    def toggle(self):
        if self.activation_state == False:
            self.activate()
        else:
            self.deactivate()

    def read_sucker_state(self):
        pressures = self.get_pressure()
        success = (pressures < self.threshold_true_vacuum_kpa) 
        not_activated = (pressures > self.threshold_not_activated_kpa)
        print("Pressures:", pressures, " | Successes: ", success, "| Not activated: ", not_activated)
        return success 


if __name__ == "__main__":
    pneumatics = Pneumatics()
    while 1:
        pneumatics.read_sucker_state()

