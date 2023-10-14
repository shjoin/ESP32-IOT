from machine import Pin, PWM, ADC, time_pulse_us
from time import sleep, sleep_us, sleep_ms
from hc_sr04 import HCSR04
import socket

"""
Class to represent our robot car
enable_pins = [21, 14]
motor_pins = [47, 48, 12, 13]
MAX_POWER_LEVEL = 65535		# 100%
MEDIUM_POWER_LEVEL = 49151  # 75%  === speed
MIN_POWER_LEVEL = 32767		# 50%

RobotCar(enable_pins, motor_pins, MEDIUM_POWER_LEVEL)
"""
class RobotCar():
    def __init__(self, enable_pins, motor_pins, In_MEDIUM_POWER_LEVEL,in_trigger_pin,in_echo_pin,in_echo_timeout_us,in_sarvo_pin,in_servo_right,in_servo_centre,in_servo_left,in_servo_delay,in_freqNo,in_MAX_POWER_LEVEL,in_MIN_POWER_LEVEL):
        print("===============================")
        print("this is Class AutoRobotCar ")        
        self.right_motor_enable_pin = PWM(Pin(enable_pins[0]), freq=2000)
        self.left_motor_enable_pin = PWM(Pin(enable_pins[1]), freq=2000)        
        self.right_motor_control_1 = Pin(motor_pins[0], Pin.OUT)
        self.right_motor_control_2 = Pin(motor_pins[1], Pin.OUT)        
        self.left_motor_control_1 = Pin(motor_pins[2], Pin.OUT)
        self.left_motor_control_2 = Pin(motor_pins[3], Pin.OUT)        
        self.mid_speed = In_MEDIUM_POWER_LEVEL
        self.max_speed=in_MAX_POWER_LEVEL
        self.min_speed=in_MIN_POWER_LEVEL        
        self.in_servo_delay=in_servo_delay
        self.in_trigger_pin=in_trigger_pin
        self.in_echo_pin=in_echo_pin
        self.in_echo_timeout_us=in_echo_timeout_us
        self.in_sarvo_pin=in_sarvo_pin
        self.in_servo_right=in_servo_right
        self.in_servo_centre=in_servo_centre
        self.in_servo_left=in_servo_left
        self.in_servo_delay=in_servo_delay
        self.in_freqNo=in_freqNo        
        #self.sensor = HCSR04(in_trigger_pin, in_echo_pin,in_echo_timeout_us)
        self.sensor = HCSR04(trigger_pin=42, echo_pin=11,echo_timeout_us=100000)
        self.pwm = PWM(Pin(in_sarvo_pin))
        self.pwm.freq(in_freqNo)
        
    def stop(self,t=0):
        print('Car stopping')
        self.right_motor_enable_pin.duty_u16(0)
        self.left_motor_enable_pin.duty_u16(0)        
        self.right_motor_control_1.value(0)
        self.right_motor_control_2.value(0)
        self.left_motor_control_1.value(0)
        self.left_motor_control_2.value(0)
        if t > 0 :
           sleep_ms(t)
        
        
    def forward(self,t=0):
        print('Move forward')       
        '''
          self.right_motor_enable_pin = PWM(Pin(enable_pins[0]), freq=2000)
        '''
        self.right_motor_enable_pin.duty_u16(self.min_speed)
        self.left_motor_enable_pin.duty_u16(self.min_speed)

        self.right_motor_control_1.value(1)
        self.right_motor_control_2.value(0)
        self.left_motor_control_1.value(1)
        self.left_motor_control_2.value(0)        
        '''
            minSpeed = 300
            midSpeed = 700
            maxSpeed = 1024
            speed = midSpeed
            PWM(Pin(14)).freq(50)
            PWM(Pin(14)).duty(speed))
        '''
        if t > 0 :
          sleep_ms(t)
           
           

    
    def reverse(self,t=0):
        print('Move reverse')
        self.right_motor_enable_pin.duty_u16(self.mid_speed)
        self.left_motor_enable_pin.duty_u16(self.mid_speed)
        
        self.right_motor_control_1.value(0)
        self.right_motor_control_2.value(1)
        self.left_motor_control_1.value(0)
        self.left_motor_control_2.value(1)
        if t > 0 :
           sleep_ms(t)
    
    def turnLeft(self,t=0):
        print('Turning Left')
        self.right_motor_enable_pin.duty_u16(self.min_speed)
        self.left_motor_enable_pin.duty_u16(self.min_speed)
        '''
        self.right_motor_control_1.value(1)
        self.right_motor_control_2.value(0)
        self.left_motor_control_1.value(0)
        self.left_motor_control_2.value(0)
    '''
        self.right_motor_control_1.value(1)
        self.right_motor_control_2.value(0)
        self.left_motor_control_1.value(0)
        self.left_motor_control_2.value(0)
        if t > 0 :
         sleep_ms(t)
        
    def turnRight(self,t=0):
        print('Turning Right')
        self.right_motor_enable_pin.duty_u16(self.min_speed)
        self.left_motor_enable_pin.duty_u16(self.min_speed)
        '''
        self.right_motor_control_1.value(0)        
        self.right_motor_control_2.value(0)
        self.left_motor_control_1.value(1)
        self.left_motor_control_2.value(0)
        '''        
        self.right_motor_control_1.value(0)        
        self.right_motor_control_2.value(0)
        self.left_motor_control_1.value(1)
        self.left_motor_control_2.value(0)
        if t > 0 :
          sleep_ms(t)
        
    def set_speed(self, new_speed):
        self.speed = new_speed
        
    def cleanUp(self):
        print('Cleaning up pins')
        self.right_motor_enable_pin.deinit()
        self.left_motor_enable_pin.deinit()
     
    def SarvoMotor(self,in_servo_duty):                    
        self.pwm.duty(in_servo_duty)         
        #sleep(1)
     
    def forward_DistanceCheck(self):         
        self.SarvoMotor(self.in_servo_centre)
        #sleep_ms(self.in_servo_delay)
        distance = self.sensor.distance_cm()        
        return round(distance,2)
    
    def right_DistanceCheck(self):         
        self.SarvoMotor(self.in_servo_right)
        sleep_ms(self.in_servo_delay)
        distance = self.sensor.distance_cm()        
        return round(distance,2)
   
    def left_DistanceCheck(self):         
        self.SarvoMotor(self.in_servo_left)
        sleep_ms(self.in_servo_delay)
        distance = self.sensor.distance_cm()        
        return round(distance,2)        
    
    def test(self):
        self.pwm.duty(self.in_servo_left)
        sleep_ms(self.in_servo_delay)
        self.pwm.duty(self.in_servo_centre)
        sleep_ms(self.in_servo_delay)
        distance = self.sensor.distance_cm()
        print('Distnace',round(distance,2))
  
        
    def autoDrive(self,In_Counter):             
         # check distance from obstacles in cm.        
        centerDistance_cm = self.forward_DistanceCheck()
        print('centerDistance Check ', centerDistance_cm)
        # then take actions in milli seconds
        if centerDistance_cm <=20:
            print('centerDistance_cm <=20 ', centerDistance_cm)
            self.stop(100)
            sleep_ms(500)            
            rightDistance_cm = self.right_DistanceCheck ()
            sleep_ms(500)            
            
            self.SarvoMotor(self.in_servo_centre) # Servo focus to center
            sleep_ms(1000)
            
            leftDistance_cm = self.left_DistanceCheck()
            sleep_ms(500)
            #centerDistance_cm = self.forward_DistanceCheck()
            #sleep_ms(500)
            self.SarvoMotor(self.in_servo_centre) # Servo focus to center
            In_Counter=In_Counter+1
            if rightDistance_cm > leftDistance_cm and rightDistance_cm >=20:
                print('turnRight leftDistance_cm ', leftDistance_cm)
                print('turnRight rightDistance_cm  ', rightDistance_cm)
                print('turnRight forward  ', centerDistance_cm)
                self.turnRight(200)
                In_Counter=1
                #sleep_ms(360)
            elif rightDistance_cm < leftDistance_cm and leftDistance_cm>=20:
                 print('turnLeft rightDistance_cm ', rightDistance_cm)
                 print('turnLeft leftDistance_cm ', leftDistance_cm)
                 self.turnLeft(200)
                 In_Counter=1
                 #sleep_ms(360)
            elif rightDistance_cm <20 or leftDistance_cm<20:
                  print('reverse rightDistance_cm <20 ' ,rightDistance_cm )
                  print('reverse leftDistance_cm ',leftDistance_cm)
                  self.reverse(200)
                  In_Counter=1
                  #sleep_ms(360)
            else:
                  self.forward()
                  In_Counter=1
                  print('function self.forward() A' )
        else:
           self.forward()
           In_Counter=1
           print('function self.forward() B' )
 
            
             