import serial

class MotorController():
    def __init__(self, port, device_number):
        """
        Don't know if we should listen on ROS topics outside
        or inside this class, probably outside
        """
        self.device_number = device_number
        try:
            self.port = serial.Serial(port)
            self.exit_safe_start()
        
        except serial.SerialException as ex:
            raise(ex)


    def set_pulse(self,speed):
        if speed < 0:
            # Motor runs in reverse
            data_byte_2 = 0x06
        else: data_byte_2 = 0x05
        speed = abs(speed)
        header = [0xAA, self.device_number, data_byte_2]
        speed_byte_1 = [speed % 32]
        speed_byte_2 = [speed / 32]
        print(header+speed_byte_1+speed_byte_2)
        try:
            self.port.write(header+speed_byte_1+speed_byte_2)
        except Exception as ex:
            print(ex)
    def close_port(self):
        try:
            self.port.close()
        except:
            print("Cannot close serial connection")
            
    def speed_to_pulse(self, speed):
        """
        Take the output from PID controller (RPM or whatever)
        and convert it to a signal the pololu can understand
        but this might be weird because it has to field angles too
        """
        raise(NotImplementedError)

    def exit_safe_start(self):
        try:
            self.port.write([0xAA,self.device_number,0x03])
            return True
        except:
            return False
