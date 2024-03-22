
import time
import smbus
import RPi.GPIO as GPIO

# Define GPIO pins for sensors
MQ135_pin = 17
MQ6_pin = 18

# Define I2C address and registers for LCD
LCD_ADDR = 0x27
LCD_WIDTH = 16
LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_BACKLIGHT = 0x08

# Define some device constants
LCD_EN = 0b00000100
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

def main():
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MQ135_pin, GPIO.IN)
    GPIO.setup(MQ6_pin, GPIO.IN)

    # Initialize LCD
    lcd_init()

    try:
        while True:
            # Read sensor values
            mq135_value = GPIO.input(MQ135_pin)
            mq6_value = GPIO.input(MQ6_pin)

            # Display values on LCD
            lcd_string("MQ135: {}".format(mq135_value), LCD_LINE_1)
            lcd_string("MQ6: {}".format(mq6_value), LCD_LINE_2)

            # Delay before next reading
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        # Clean up GPIO
        GPIO.cleanup()

def lcd_init():
    # Initialise display
    lcd_byte(0x33, LCD_CMD)  # 110011 Initialise
    lcd_byte(0x32, LCD_CMD)  # 110010 Initialise
    lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
    lcd_byte(0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
    lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = data
    # mode = True  for character
    #        False for command

    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus_write_byte(bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus_write_byte(bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # Toggle enable
    time.sleep(E_DELAY)
    bus_write_byte(bits | ENABLE)
    time.sleep(E_PULSE)
    bus_write_byte(bits & ~ENABLE)
    time.sleep(E_DELAY)

def bus_write_byte(bits):
    # Send byte to I2C bus
    # bits = data
    # bits = mode
    byte = bits | LCD_BACKLIGHT
    bus.write_byte(LCD_ADDR, byte)

def lcd_string(message, line):
    # Send string to display
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

if __name__ == '__main__':
    main()
    
