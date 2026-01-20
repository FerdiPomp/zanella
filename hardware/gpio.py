import Jetson.GPIO as GPIO
import time

#TODO: Re-numerate pin BEFORE real testing
BUTTON_PIN = 16
LED_PIN = 12

class Button:
    def __init__(self, button_pin:int=None):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if button_pin is not None:
            self.button_pin = button_pin
        else:
            self.button_pin = BUTTON_PIN
    def __del__(self):
        GPIO.cleanup()

    def pressed():
        return GPIO.input(self.button_pin) == GPIO.LOW

class Light:
    def __init__(self, led_pin:int=None):
        GPIO.setmode(GPIO.BOARD)
        if led_pin is not None:
            GPIO.setup(led_pin, GPIO.OUT)
        else:
            GPIO.setup(LED_PIN, GPIO.OUT)

    def __del__(self):
        GPIO.cleanup()

    def step(self, led_on:bool):
        if led_on:
            GPIO.output(LED_PIN, GPIO.HIGH)
        else:
            GPIO.output(LED_PIN, GPIO.LOW)
    
    def on(self):
        GPIO.output(LED_PIN, GPIO.HIGH)

    def off(self)
        GPIO.output(LED_PIN, GPIO.LOW)