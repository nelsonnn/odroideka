# Uses the pySerial library to send and receive data from a Jrk G2.
#
# NOTE: The Jrk's input mode must be "Serial / I2C / USB".
# NOTE: You might need to change the "port_name =" line below to specify the
#   right serial port.
 
import serial
 
class JrkG2Serial(object):
  def __init__(self, port, device_number=None):
    self.port = port
    self.device_number = device_number
 
  def send_command(self, cmd, *data_bytes):
    if self.device_number == None:
      header = [cmd]  # Compact protocol
    else:
      header = [0xAA, device_number, cmd & 0x7F]  # Pololu protocol
    self.port.write(header + list(data_bytes))
 
  # Sets the target.  For more information about what this command does,
  # see the "Set Target" command in the "Command reference" section of
  # the Jrk G2 user's guide.
  def set_target(self, target):
    self.send_command(0xC0 + (target & 0x1F), (target >> 5) & 0x7F)
 
  # Gets one or more variables from the Jrk (without clearing them).
  def get_variables(self, offset, length):
    self.send_command(0xE5, offset, length)
    result = self.port.read(length)
    if len(result) != length:
      raise RuntimeError("Expected to read {} bytes, got {}."
        .format(length, len(result)))
    return bytearray(result)
 
  # Gets the Target variable from the Jrk.
  def get_target(self):
    b = self.get_variables(0x02, 2)
    return b[0] + 256 * b[1]
 
  # Gets the Feedback variable from the Jrk.
  def get_feedback(self):
    b = self.get_variables(0x04, 2)
    return b[0] + 256 * b[1]
 
# Choose the serial port name.  If the Jrk is connected directly via USB,
# you can run "jrk2cmd --cmd-port" to get the right name to use here.
# Linux USB example:  "/dev/ttyACM0"
# macOS USB example:  "/dev/cu.usbmodem001234562"
# Windows example:    "COM6"
port_name = "/dev/ttyACM0"
 
# Choose the baud rate (bits per second).  This does not matter if you are
# connecting to the Jrk over USB.  If you are connecting via the TX and RX
# lines, this should match the baud rate in the Jrk's serial settings.
baud_rate = 9600
 
# Change this to a number between 0 and 127 that matches the device number of
# your Jrk if there are multiple serial devices on the line and you want to
# use the Pololu Protocol.
device_number = None
 
port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)
 
jrk = JrkG2Serial(port, device_number)
 
feedback = jrk.get_feedback()
print("Feedback is {}.".format(feedback))
 
target = jrk.get_target()
print("Target is {}.".format(target))
 
new_target = 2248 if target < 2048 else 1848
print("Setting target to {}.".format(new_target))
jrk.set_target(new_target)
