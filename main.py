from machine import Pin, UART, I2C
#ref.: https://www.youtube.com/watch?v=y2EDhzDiPTE
#ref.: https://github.com/ahmadlogs/rpi-pico-upy/tree/main/gps2-rpi-pico

#Import utime library to implement delay
import utime, time

#________________________________________________________
from ssd1306 import SSD1306_I2C
#https://github.com/stlehmann/micropython-ssd1306

import onewire, ds18x20

onboard_led = Pin("LED", Pin.OUT)
onboard_led.on()

heater_led = Pin(15, Pin.OUT)
heater_led.off()

##########################################################
#Oled I2C connection
#i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
#oled = SSD1306_I2C(128, 64, i2c)

#oled.fill(0)
#oled.text("No Data", 0, 0)
#oled.show()
#utime.sleep(1)

ds_pin = machine.Pin(18)
 
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

roms = ds_sensor.scan()

print('Found DS devices: ', roms)

WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height

# sda=machine.Pin(20)
# scl=machine.Pin(21)
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

oled.fill(0)
oled.text("No Data", 0, 0)
oled.show()

push_button = Pin(22, Pin.IN)  # 22 number pin is input

solid_state_relay = Pin(28, Pin.OUT)
solid_state_relay.value(0)

try:
    while True:
        logic_state = push_button.value()

        ds_sensor.convert_temp()
        time.sleep_ms(750)
        
        for rom in roms:
            print(rom)
            print(ds_sensor.read_temp(rom))

        # Clear the oled display in case it has junk on it.
        oled.fill(0)       
        
        # Add some text
        oled.text("Temperature",20,8)
        oled.text(str(round(ds_sensor.read_temp(rom),1)),36,24)
        oled.text("*C",76,24)
        oled.text("Heater",40,40)
        if logic_state == True:
            heater_led.on()
            oled.text("ON",56, 56)
            solid_state_relay.value(1)
        else:
            solid_state_relay.value(0)
            oled.text("OFF",52, 56)
            heater_led.off()

            
        # Finally update the oled display so the image & text is displayed
        oled.show()
        utime.sleep(2)
        
finally:    
    # Clear the oled display in case it has junk on it.
    solid_state_relay.value(0)
    oled.fill(0)
    oled.show()
    onboard_led.off()
    heater_led.off()


