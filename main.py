from machine import Pin, UART, I2C
#ref.: https://www.youtube.com/watch?v=y2EDhzDiPTE
#ref.: https://github.com/ahmadlogs/rpi-pico-upy/tree/main/gps2-rpi-pico

#Import utime library to implement delay
import utime, time
import network
import sys

from do_connect import *

from umqtt.simple import MQTTClient

#________________________________________________________
from ssd1306 import SSD1306_I2C
#https://github.com/stlehmann/micropython-ssd1306

import onewire, ds18x20

from secrets import secrets

onboard_led = Pin("LED", Pin.OUT)
onboard_led.on()

heater_led = Pin(15, Pin.OUT)
heater_led.off()

# def beanaproblem(string):
#     refresh(ssd, True)  # Clear any prior image
#     relaypin = Pin(15, mode = Pin.OUT, value =0 )
#     utime.sleep(2)

# Look for thermometer (add OLED complaint if one can't be seen)
ds_pin = Pin(18)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
roms = ds_sensor.scan()
print('Thermometer: ', roms)
if roms=='':
    print('No Thermometer. STOP')
    # beanaproblem('No Therm.')
    utime.sleep(60)
    sys.exit()


WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height

sda=Pin(8)
scl=Pin(9)
i2c = I2C(0, sda=sda, scl=scl, freq=200000)

print('Scan i2c bus...')
devices = i2c.scan()

if len(devices) == 0:
    print("No i2c device !")
else:
    print('i2c devices found:',len(devices))

for device in devices:
    print("Decimal address: ",device," | Hexa address: ",hex(device))

print("I2C Address      : "+hex(i2c.scan()[0]).upper()) # Display device address
print("I2C Configuration: "+str(i2c))                   # Display I2C config

oled = SSD1306_I2C(WIDTH, HEIGHT, i2c)                  # Init oled display

oled.rotate(True)

oled.fill(0)
oled.text("Battery Heater", 0, 0)
oled.show()

##########################################################
#MQTT  Topic Setup
def connectMQTT():
    #https://forum.micropython.org/viewtopic.php?t=11076&p=60836
    client = MQTTClient(secrets['client_id'],secrets['mqtt_server'], keepalive=30) # Just added Keepalive=30 to fix the error code 2
    client.connect()
    return client

def publishMQTT(topic, value):
    # print(topic)
    # print(value)
    # pub_msg = "%5.2f" % value
    print("Publish: ",topic,"  ",value)
    # client.publish(topic, pub_msg)
    client.publish(topic, value)
    #print("publish Done")

# WiFi connection
ip = do_connect()

oled.fill(0)
oled.text("IP Address", 0, 0)
oled.text(ip, 0, 16)
oled.rotate(True)
oled.show()
utime.sleep(2.0)

try:
    client = connectMQTT()
except OSError as e:
    oled.fill(0)
    oled.text('ERROR: connectMQTT', 0, 0)
    oled.show()
    utime.sleep(2.0)


push_button = Pin(22, Pin.IN)  # 22 number pin is input

relaypin = Pin(28, Pin.OUT)
relaypin.value(0)


# Main Logic
pin = 0
counter = 8.0 # target temperature
integral = 0
lastupdate = utime.time()
#refresh(ssd, True)  # Initialise and clear display.

lasterror = 0
# The Tweakable values that will help tune for our use case. TODO: Make accessible via menu on OLED
checkin = 10
# Stolen From Reddit: In terms of steering a ship:
# Kp is steering harder the further off course you are,
# Ki is steering into the wind to counteract a drift
# Kd is slowing the turn as you approach your course
Kp = 20.   # Proportional term - Basic steering (This is the first parameter you should tune for a particular setup)
Ki = .01   # Integral term - Compensate for heat loss by vessel
Kd = 150.  # Derivative term - to prevent overshoot due to inertia - if it is zooming towards setpoint this
           # will cancel out the proportional term due to the large negative gradient
P = 0.
I = 0.
D = 0.

output = 0
offstate = True
boil = False  # The override flag that will just get to a boil as quick as possible. (Assumes water at sea level, which is ok for now)
# Heating loop - Default behaviour
powerup = True

try:
    while True:
#        logic_state = push_button.value()

        # counter=encoder(pin)
        # # If the counter is set to 100 and we assume we're heating water, 100 degrees is as hot as the water can get,
        # # so the output should just be set to 100 until the target is reached. Much quicker for this use case.
        # if counter==100:
        #     boil = True
        # else:
        #     boil = False
        ds_sensor.convert_temp()
        temp = ds_sensor.read_temp(roms[0])

        now = utime.time()
        dt = now - lastupdate

        if output < 100 and offstate == False and dt > checkin * round(output)/100 :
            relaypin.value(0)
            heater_led.off()
            offstate = True
            utime.sleep(.1)
        if dt > checkin:
            error = counter - temp
            integral = integral + dt * error
            derivative = (error - lasterror) / dt
            P = Kp * error
            I = Ki * integral
            D = Kd * derivative
            output = P + I + D
            output = max(min(100, output), 0) # Clamp output between 0 and 100

            print("T: %5.2f, G: %5.2f, E: %5.2f, O: %6.2f, P: %7.4f, I: %7.4f, D: %7.4f" % (temp, counter, error, output, P, I, D))
            publishMQTT('battery/temp','{"temp":'+str(temp)+'}')

            if output > 0:
                relaypin.value(1)
                heater_led.on()
                offstate = False
            else:
                relaypin.value(0)
                heater_led.off()
                offstate = True
            utime.sleep(.1)
            lastupdate = now
            lasterror = error

        # Clear the oled display in case it has junk on it.
        oled.fill(0)
        oled.text("IP: "+ip, 0, 0)
        oled.text("Target : "+str(round(counter,1))+" *C",0,9)
        oled.text("Current: "+str(round(temp,1))+" *C",0,18)
        oled.text("Heater : "+str(round(output))+"%",0,27)
        if offstate == False:
            oled.text("ON",112, 27)
        else:
            oled.text("OFF",104, 27)
        oled.text("P --> "+str(P),0,36)
        oled.text("I --> "+str(I),0,45)
        oled.text("D --> "+str(D),0,54)
        # Finally update the oled display so the image & text is displayed
        oled.show()
        
finally:    
    # Clear the oled display in case it has junk on it.
    relaypin.value(0)
    oled.fill(0)
    oled.show()
    heater_led.off()
    onboard_led.off()
#    machine.reset()
