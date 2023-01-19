import time
from machine import Pin, PWM, ADC

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
        current_sensor_pin,
        current_limit = 5
    ):
        self.FWD0_REV1 = Pin(FWD0_REV1_pin, Pin.OUT)
        self.ON = Pin(ON_OFF_pin, Pin.OUT)
        self.ON.value(0)
        self.pwm = PWM(Pin(mot_pot_pin))

        self.pwm.freq(50)
        self.servo_position = self.uMIN  # intialize  speed as 0
        self.pwm.duty_ns(self.servo_position)  #   set mot pot to zero

        self.current_sensor = ADC(Pin(current_sensor_pin, mode=Pin.IN))
        self.current_limit = current_limit

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

    def measure_current(self):
        # Measure current draw of motor. Shut off if above threshold
        voltage_divider = (100+47)/100
        if self.ON.value()==1:
            volty = self.current_sensor.read_u16()* 3.3 / 65535
            curry = (-10 * volty * voltage_divider + 25)
            vs = '%.2f' % volty
            cs = '%.2f' % curry
            print(vs,"V ; ",cs, "A")

            if abs(curry) > self.current_limit:
                for i in range(4): # double check before shutoff -- take an average over 50 ms
                    time.sleep_ms(10)
                    volty = self.current_sensor.read_u16()* 3.3 / 65535
                    curry = curry - 10 * volty * voltage_divider + 25
                curry = curry/(i+2)
                if abs(curry) > self.current_limit:
                    self.ON.value(0)
                    self.move_servo(0)
#                     level_wind.writeSpeed(0)
                    print("HIGH CURRENT (",str(curry),"A ) DETECTED! shutting off motor...")
