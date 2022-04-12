import RPi.GPIO as GPIO
import time
lamp_state_path = "/home/eaibot/dashgo_ws/src/uv_robots/robot-app-node/src/modules/lamp/lamp_state.txt"
GPIO.setmode(GPIO.BCM)
RELAIS_1_GPIO = 26
GPIO.setup(RELAIS_1_GPIO, GPIO.OUT) # GPIO Assign mode
GPIO.output(RELAIS_1_GPIO, GPIO.LOW) # out
time.sleep(0.2)
lamp_state = "off"
while(True):
    f = open(lamp_state_path, "r")
    lamp_state = f.read().strip('\n')
    if(lamp_state == "off"):
        GPIO.output(RELAIS_1_GPIO, GPIO.LOW) # out
        time.sleep(0.2)
      
    elif(lamp_state =="on"):
        GPIO.output(RELAIS_1_GPIO, GPIO.HIGH) # out
        time.sleep(0.2)
        lamp_state = True
    

        
