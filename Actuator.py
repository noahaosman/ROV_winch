import time
from machine import Pin, PWM, I2C

# constants for actuator
RETRACT = 1
EXTEND = 0
false_pulse_delay_actuator = 2 # (zero for no debounce delay)
pulses_per_inch = 54
false_pulse_delay_reed_sw = 250 # ms

class Actuator:

    def __init__(
        self,
        scl_pin,
        sda_pin,
        ON_OFF_pin,
        direction_pin,
        feedback_pin,
        rotation_pin,
        cable_diameter = 0.375
    ):

        self.i2c = I2C(
            1,
            scl=Pin(scl_pin),
            sda=Pin(sda_pin))
        self.ON = Pin(ON_OFF_pin, Pin.OUT)
        self.direction = Pin(direction_pin, Pin.OUT)
        self.current_direction = EXTEND

        self.feedback = Pin(feedback_pin, Pin.IN, Pin.PULL_UP)
        self.feedback.irq(trigger=Pin.IRQ_RISING, handler=self.updatePosition)
        self.last_pulse_time = time.ticks_ms()-false_pulse_delay_actuator
        self.position = 0

        self.reed_sw = Pin(rotation_pin, Pin.IN, Pin.PULL_DOWN)
        self.reed_sw.irq(trigger=Pin.IRQ_RISING, handler=self.rotationTrackingReedSwitchTrigger)
        self.last_reed_time = time.ticks_ms()-false_pulse_delay_reed_sw
        self.NeedToMoveActuator = False

        self.ON.value(0)

        self.cable_diameter = cable_diameter

        self.MAspeed = 100

        self.time_init = time.ticks_ms()*0.001

        with open('stacking_state.txt') as infp:
            self.line_stack_state = int(infp.read())

# update actuator position
    def updatePosition(self, p):
        current_pulse_time = time.ticks_ms()
        if (current_pulse_time - self.last_pulse_time ) > false_pulse_delay_actuator: # debouncing
            self.position = self.position + 1
###            print("position: ", self.position," TBP: ", (current_pulse_time - self.last_pulse_time )*0.001)
            self.last_pulse_time = current_pulse_time



# write speed to actuator. 0<=value<=100
    def writeSpeed(self, value):
        if value == 0:
            self.ON.value(0)
        else:
            self.ON.value(1)

        value = round(value/100*4095)
        buf=bytearray(2)
        buf[0]=(value >> 8) & 0xFF
        buf[1]=value & 0xFF
        self.i2c.writeto(0x60,buf)


# define actuator speed as a function of the winch speed and distance to move
    def getSpeed(self, winch_speed, distance, manual_adjust):
        if manual_adjust: # if maunally adjusting, base speed off distance to move
            if distance<0.75:
                speed = 20
            else:
                speed = 100
        else: # else if winching, base speed off winch speed
            x2 = self.calculateSpeed(distance, winch_speed, 0.33)
            speed = x2
        speed = round(max(25, min(100, speed))) # bound speed from 20 <-> 100
###        print("Actuator speed: ", speed)
        return speed


# move actuator some distance in inches
    def move(self, winch_speed, direction = None, distance = None, manual_adjust = False): # default to cable diameter
        if direction is None:
            direction = self.current_direction
        if distance is None:
            distance = self.cable_diameter
        self.position = 0   # zero position tracker
        self.direction.value(direction)
        speed = self.getSpeed(winch_speed, distance, manual_adjust)
        self.writeSpeed(speed)  # write a speed
        self.time_init = time.ticks_ms()*0.001
        self.last_pulse_time = self.time_init*1000
        target_pulses = (round(pulses_per_inch*distance - self.Overshoot(speed))) # add some overshoot for time to turn off
        stationary_counter = 0
        start_time = time.ticks_ms()
        loop_start = start_time
###        print("========== New move loop. Time: ",start_time*0.001-self.time_init, ". Target pulses: ",round(pulses_per_inch*distance)," ==========")
        # check counted pulses every 50 ms.
        while True:
            prior_position = self.position
            time.sleep(0.05)  # wait for actuator to move
            loop_end = time.ticks_ms()
###            print("-------------loop------------- Time: ", (loop_end-loop_start)*0.001)
            loop_start = loop_end
            if self.position >= target_pulses:  # if we've reached our target,
###                print("target reached. Total time: ", (time.ticks_ms()-start_time)*0.001)
                self.writeSpeed(0)  # turn off actuator
                self.direction.value(self.opposite(direction)) # bounce back
                break
            elif self.position == prior_position:  # if actuator is not moving
                stationary_counter = stationary_counter + 1
                if stationary_counter > 10:
###                    print("edge detected")
                    self.writeSpeed(0)  # turn off actuator
                    if not manual_adjust: # reverse direction only if currently winching
                        self.changeDirection()
                    break
            else:
                stationary_counter = 0


# change actuator direction
    def changeDirection(self):
###        print("reversing direction")
        self.current_direction = self.opposite(self.current_direction)
        self.direction.value(self.current_direction) # reverse direction
        self.line_stack_state = self.opposite(self.line_stack_state)
        with open('stacking_state.txt') as infp:
            infp.write(str(self.line_stack_state))


# reed switch irq for rotation tracking
    def rotationTrackingReedSwitchTrigger(self,p):
###        print("!!ah!!")
        current_reed_time = time.ticks_ms()
        if (current_reed_time - self.last_reed_time ) > false_pulse_delay_reed_sw:
            time.sleep_ms(5)
###            print("**5ms**")
            if self.reed_sw.value() == 1:
                self.last_reed_time = current_reed_time
                self.NeedToMoveActuator = True  # move actuator one cable width
###                print("rotation reed switch triggered, time: ",current_reed_time*0.001-self.time_init)


# a useful function to flip booleans, ie 0 --> 1 and 1 --> 0
    def opposite(self, input):
        out = 1 - input
        return out


# actuator tick overshoot as a function of actuator speed
    def Overshoot(self, x):
        c0 = 0.0157003
        c1 = 0.0705877
        c2 = -0.000215076
        out = c0 + c1*x + c2*x**2
        return out

# return optimal actuator speed for a given distance, winch speed, % rotation time for actuation
    def calculateSpeed(self, x, winch_speed, perc):
        # actuator speed in in/s = c0 + c1*P + c2*P^2
        c0 = 0.054397
        c1 = 0.0178693
        c2 = -0.0000678376
        SPR = self.secondsPerRotation(winch_speed)
        T = perc*SPR
        P = (-c1 + ( c1**2 + 4*c2*(x/T - c0) )**0.5 ) / (2*c2)
        return P

# seconds per spool rotation as a function of motor speed
    def secondsPerRotation(self, x):
        rpm = -11.8876 + 0.372714*x + 0.00207748*x**2.0
        out = 60/rpm
        return out
