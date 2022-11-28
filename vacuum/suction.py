# -*- coding:utf-8 -*-

import serial
import time
import numpy as np
class PythonSerialDriver():
    def __init__(self, suction_control_port="/dev/ttyUSB0", suction_read_port="/dev/ttyUSB1"):
        self.windows = False
        self.activation_state = False
        # (38400Kbit/s, 8,N,1)
        suction_control_baud = 9600
        self.ser_control = None
        self.ser_control = serial.Serial(suction_control_port, suction_control_baud, timeout=0.1) # 1s timeout
        self.ser_control.reset_input_buffer()
        self.ser_control.reset_output_buffer()         

        if self.ser_control.is_open:
            print("suction_control_port open success")
        else:
            print("suction_control_port open failed")

        suction_read_baud = 19200
        self.ser_read = None
        self.ser_read = serial.Serial(suction_read_port, suction_read_baud, timeout=0.01) # 1s timeout
        self.ser_read.reset_input_buffer()
        self.ser_read.reset_output_buffer()         

        if self.ser_read.is_open:
            print("suction_read_port open success")
        else:
            print("suction_read_port open failed")

    
    def send_data(self, send_data):
        send_data = bytes.fromhex(send_data)
        self.ser_control.write(send_data)   # 发送命令
        time.sleep(0.1)        # 延时，否则len_return_data将返回0，此处易忽视！！！

    def activate(self):
        self.send_data('01050000FF008C3A')
        self.send_data('01050001FF00DDFA')
        self.send_data('01050002FF002DFA')
        self.send_data('01050003FF007C3A')
        self.activation_state = True
        print("Activate suction cup")
        # time.sleep(3)        # 延时，否则len_return_data将返回0，此处易忽视！！！
    
    def deactivate(self):
        self.send_data('010500000000CDCA')
        self.send_data('0105000100009C0A')
        self.send_data('0105000200006C0A')
        self.send_data('0105000300003DCA')
        self.activation_state = False
        print("Deactivate suction cup")
        # time.sleep(1)        # 延时，否则len_return_data将返回0，此处易忽视！！！
    
    def toggle(self):
        if self.activation_state == False:
            self.activate()
        else:
            self.deactivate()

    def read_value(self):
        # [12][03][00][11][00][02][96][AD]
        self.ser_read.reset_input_buffer()
        self.ser_read.reset_output_buffer()         
        self.send_data('12030011000296AD')
        time.sleep(0.1)
        # line = self.ser_read.readline()
        line=self.ser_read.read(8)
        try:
            # value = float(str(line).split("b'-12,10,")[1].split(",")[0].split('\\n')[0])
            value=''
            print("Read line:", line, value)
            
        except:
            print("read_value fail, retrying")
            # time.sleep(0.01)
            value=np.nan

        return value

    def read_sucker_state(self):
        values=[]
        for i in range(10):
            values = np.append(values, self.read_value())
        values = values[~np.isnan(values)]
        value = np.average(values)

        if value < -55:
            # success
            print("Grip success")
            state=1
        elif value > 60:
            # Not gripped
            print("Not activated")
            state=0
        else:
            # Nothing gripped
            print("Activated but no grip")
            state=0
            
        return state

if __name__ == "__main__":
    srt = PythonSerialDriver()
    timenow = time.time()
    srt.deactivate()
    while 1:
        print(srt.read_value())
        # srt.read_sucker_state()
        # Toggle suction        
        # if time.time() - timenow > 1:
        #     print("1 sec passed")
        #     timenow = time.time()
        #     srt.toggle()
