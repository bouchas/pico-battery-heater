from machine import Pin, UART, I2C
#ref.: https://www.youtube.com/watch?v=y2EDhzDiPTE
#ref.: https://github.com/ahmadlogs/rpi-pico-upy/tree/main/gps2-rpi-pico

#Import utime library to implement delay
import utime, time
import network
import sys

#from umqtt.simple import MQTTClient

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

sda=machine.Pin(8)
scl=machine.Pin(9)
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


#network declaration
# Set country to avoid possible errors / https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
rp2.country('CA')

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

#connect using ssid
wlan.connect(secrets['ssid'],secrets['password'])
while not wlan.isconnected():
    #machine.idle() # save power while waiting
    print('Waiting for connection...')
    utime.sleep(1.0)
ip = wlan.ifconfig()[0]
print(f'IP Address: {ip}')

oled.fill(0)
oled.text("IP: "+ip, 0, 0)
oled.show()

# oled.poweroff()
# oled.poweron()

push_button = Pin(22, Pin.IN)  # 22 number pin is input

relaypin = Pin(28, Pin.OUT)
relaypin.value(0)


# Main Logic
pin = 0
counter = 30 # target temperature
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
#             output = Kp * error + Ki * integral + Kd * derivative
            output = P + I + D
#             print(str(output)+"= Kp term: "+str(Kp*error)+" + Ki term:" + str(Ki*integral) + "+ Kd term: " + str(Kd*derivative))
            print("Temp: " + str(temp))
            print("SV: " + str(counter))
            print("Error: " + str(error))
            print("MV: ")
            print("P ----> " + str(P))
            print("I ----> " + str(I))
            print("D ----> " + str(D))

            output = max(min(100, output), 0) # Clamp output between 0 and 100
            if boil and error > .5:
                output=100
            print(output)
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
        # Add some text
        oled.text("IP: "+ip, 0, 0)
        oled.text("Temperature",20,16)
        #print("temp: "+str(temp))
        oled.text(str(round(temp,1)),36,24)
        oled.text("*C",76,24)
        oled.text("Heater",40,40)
        if offstate == False:
            oled.text("ON",56, 56)
        else:
            oled.text("OFF",52, 56)
        # Finally update the oled display so the image & text is displayed
        oled.show()
        
finally:    
    # Clear the oled display in case it has junk on it.
    relaypin.value(0)
    oled.fill(0)
    oled.show()
    heater_led.off()
    onboard_led.off()

