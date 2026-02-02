import Jetson.GPIO as GPIO
import time
import config as CONFIG
import sys

class Button:
    def __init__(self, button_pin:int=CONFIG.BUTTON_PIN):
        self.button_pin = button_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    def __del__(self):
        GPIO.cleanup()

    def pressed():
        return GPIO.input(self.button_pin) == GPIO.LOW

class Light:
    def __init__(self, led_pin:int=CONFIG.LED_PIN):
        self.led_pin = led_pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_pin, GPIO.OUT)
        

    def __del__(self):
        GPIO.cleanup()

    def step(self, led_on:bool):
        if led_on:
            GPIO.output(self.led_pin, GPIO.HIGH)
        else:
            GPIO.output(self.led_pin, GPIO.LOW)
    
    def on(self):
        GPIO.output(self.led_pin, GPIO.HIGH)

    def off(self):
        GPIO.output(self.led_pin, GPIO.LOW)

