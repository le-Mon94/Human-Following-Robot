from pyfirmata import Arduino, PWM

# Motor Class
class Motor:
    def __init__(self, board, ena_pin, in1_pin, in2_pin):

        self.ena = board.get_pin(f'd:{ena_pin}:p')
        self.in1 = board.get_pin(f'd:{in1_pin}:o')
        self.in2 = board.get_pin(f'd:{in2_pin}:o')
        
    def forward(self, speed):

        self.in1.write(1)
        self.in2.write(0)
        self.ena.write(speed)
        
    def backward(self, speed):

        self.in1.write(0)
        self.in2.write(1)
        self.ena.write(speed)

motor_pins = {
    'motor1': {'ena': 11, 'in1': 13, 'in2': 12},
    'motor2': {'ena': 10, 'in1': 9, 'in2': 8},
    'motor3': {'ena': 5, 'in1': 7, 'in2': 6},
    'motor4': {'ena': 3, 'in1': 4, 'in2': 2}
}