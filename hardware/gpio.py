import config as CONFIG


GPIO_CHIP = "/dev/gpiochip0"


class Button:
    def __init__(self):
        import gpiod
        from gpiod.line import Direction

        self.gpiod = gpiod
        self.button_pin = CONFIG.BUTTON_PIN
        self.line_request = self.gpiod.request_lines(
            GPIO_CHIP,
            consumer="Button_line",
            config={self.button_pin: self.gpiod.LineSettings(direction=Direction.INPUT)},
        )

    def __del__(self):
        if hasattr(self, "line_request"):
            self.line_request.release()

    def pressed(self):
        value = self.line_request.get_value(self.button_pin)
        return value == value.ACTIVE


class Light:
    def __init__(self):
        import gpiod
        from gpiod.line import Direction, Value

        self.gpiod = gpiod
        self.value = Value
        self.led_pin = CONFIG.LED_PIN
        self.line_request = self.gpiod.request_lines(
            GPIO_CHIP,
            consumer="Led_line",
            config={self.led_pin: self.gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)},
        )

    def __del__(self):
        if hasattr(self, "line_request"):
            self.line_request.release()

    def on(self):
        self.line_request.set_value(self.led_pin, self.value.ACTIVE)

    def off(self):
        self.line_request.set_value(self.led_pin, self.value.INACTIVE)
