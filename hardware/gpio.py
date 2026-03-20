import gpiod
from gpiod.line import Direction, Value
import time
import config as CONFIG
import sys

class Button:
    def __init__(self, button_pin:int=CONFIG.BUTTON_PIN):
        self.button_pin = button_pin
        self.request_in = gpiod.request_lines(
            "/dev/gpiochip0",
            consumer="Button_line",
            config={
                self.button_pin: gpiod.LineSettings(direction=Direction.INPUT)}
        )  

    def __del__(self):
        self.request_in.release()

    def pressed(self):
        value = self.request_in.get_value(self.button_pin)
        return value == value.ACTIVE

class Light:
    def __init__(self, led_pin:int=CONFIG.LED_PIN):
        self.led_pin = led_pin
        self.request_out = gpiod.request_lines(
            "/dev/gpiochip0",
            consumer="Led_line",
            config={self.led_pin : gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)}
        )

    def __del__(self):
        self.request_out.release()
    
    def on(self):
        self.request_out.set_value(self.led_pin , Value.ACTIVE)

    def off(self):
        self.request_out.set_value(self.led_pin , Value.INACTIVE)

