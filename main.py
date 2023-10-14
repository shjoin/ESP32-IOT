from AutoRobotcar import RobotCar
from time import sleep, sleep_us, sleep_ms
from machine import Pin, PWM, ADC, time_pulse_us,UART
from hc_sr04 import HCSR04
import BLE

print("============this is MAIN.py================")


# Wifi Robot Car Configuration
MAX_POWER_LEVEL = 65535		# 100%
MEDIUM_POWER_LEVEL = 49151  # 75%
MIN_POWER_LEVEL = 32767		# 50%

enable_pins = [21, 14]
motor_pins = [47, 48, 12, 13]
trigger_pin=42
echo_pin=11
echo_timeout_us=100000

sarvo_pin=18
freqNo=50
servo_right = 54  
servo_centre = 90 #/90
servo_left = 126  #
servo_delay = 1000 #ms
Counter=1

robot_car = RobotCar(enable_pins, motor_pins, MEDIUM_POWER_LEVEL,trigger_pin,echo_pin,echo_timeout_us,sarvo_pin,servo_right,servo_centre,servo_left,servo_delay,freqNo,MAX_POWER_LEVEL,MIN_POWER_LEVEL)
UART = machine.UART(0, baudrate=115200)

#BLE.demo()

# while True:    
#   robot_car.autoDrive(Counter)

'''
robot_car.SarvoMotor(servo_right)
sleep_ms(servo_delay)
robot_car.SarvoMotor(servo_centre)
sleep_ms(servo_delay)
robot_car.SarvoMotor(servo_left)
'''
'''
pwm = PWM(Pin(sarvo_pin))
pwm.freq(freqNo)        
pwm.duty(servo_right)
sleep_ms(servo_delay)
pwm.duty(servo_centre)
sleep_ms(servo_delay)
pwm.duty(servo_left)
        
#dist1=robot_car.forward_DistanceCheck()
#print('forward',dist1)

'''
'''
sensor = HCSR04(trigger_pin=42, echo_pin=11,echo_timeout_us=1000000)
pwm = PWM(Pin(sarvo_pin))
pwm.freq(freqNo)

while True:
    sleep_ms(servo_delay)
    centerDistance_cm = sensor.distance_cm()        
    print('forward-center ', centerDistance_cm)
    sleep_ms(servo_delay)
 '''
'''
sensor = HCSR04(trigger_pin=42, echo_pin=11)
pwm = PWM(Pin(sarvo_pin))
pwm.freq(freqNo)
'''

'''
pwm.duty(servo_centre) # Servo focus to center
sleep_ms(servo_delay) 
pwm.duty(servo_left) # Servo focus to center
sleep_ms(servo_delay) 
pwm.duty(servo_centre) # Servo focus to center
sleep_ms(servo_delay) 
pwm.duty(servo_right) # Servo focus to center
sleep_ms(servo_delay) 
pwm.duty(servo_centre) # Servo focus to center
'''

'''
while True:
    
    pwm.duty(servo_centre) # Servo focus to center
    sleep_ms(servo_delay)
    centerDistance_cm = sensor.distance_cm()        
    print('forward ', centerDistance_cm)    #sleep_ms(servo_delay)    
    pwm.duty(servo_right)
    sleep_ms(servo_delay)            
    rightDistance_cm = sensor.distance_cm()
    print('rightDistance_cm ',rightDistance_cm)
    sleep_ms(servo_delay)                 
    pwm.duty(servo_centre)E # Servo focus to center
    centerDistance_cm = sensor.distance_cm()        
    print('forward-2 ', centerDistance_cm)    
    sleep_ms(servo_delay)                 
    pwm.duty(servo_left)
    sleep_ms(servo_delay)            
    leftDistance_cm = sensor.distance_cm()
    print('leftDistance_cm ',leftDistance_cm)
    sleep_ms(servo_delay) 
'''
   


