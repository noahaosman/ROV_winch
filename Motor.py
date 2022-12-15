import time
from machine import Pin, PWM

class Motor:

    # constants for motorized potentiometer
    uMIN = 500000
    uMAX = 1320000# full 270 deg of motion = 2550000
    uSTEP = int((uMAX-uMIN)/100)

    def __init__(
        self,
        FWD0_REV1_pin,
        ON_OFF_pin,
        mot_pot_pin,
    ):
        self.FWD0_REV1 = Pin(FWD0_REV1_pin, Pin.OUT)
        self.ON = Pin(ON_OFF_pin, Pin.OUT)
        self.pwm = PWM(Pin(mot_pot_pin))

        self.pwm.freq(50)
        self.servo_position = self.uMIN  # intialize winch speed as 0
        self.pwm.duty_ns(self.servo_position)  #   set mot pot to zero

    def move_servo(self, percent):
        move_to = int(self.uMIN + (self.uMAX-self.uMIN)*(percent/100))
        if move_to>=self.servo_position:
            dir = 1
        else:
            dir = -1
        for i in range(self.servo_position,move_to,dir*self.uSTEP):
            self.pwm.duty_ns(i)
            time.sleep(0.01)
        self.servo_position = move_to

    def seconds_per_rotation(x):
        rpm = -11.8876 + 0.372714*x + 0.00207748*x**2.0
        out = 60/rpm
        return out
