import board
import neopixel
import time
import math
import threading
import asyncio
import websockets
import json
from RpiMotorLib import rpi_dc_lib
 
# NeoPixel Setup
ORDER = neopixel.GRB
pixel_pin = board.D10
num_pixels = 30
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, auto_write=False, pixel_order=ORDER)

# Motor Setup
MotorLeft = rpi_dc_lib.L298NMDc(19, 13, 26, 50, True, "motor_left")
MotorRight = rpi_dc_lib.L298NMDc(6, 5, 12, 50, True, "motor_right")

# Global Variables
status = 2  # Default to standby
movement_detected = False
last_movement_time = time.time()

import time
import Adafruit_PCA9685

# Initialize the PCA9685 using the default address (0x40) and busnum (1 for /dev/i2c-1)
pwm = Adafruit_PCA9685.PCA9685(busnum=1)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

pan1 = 0
tilt1 = 0
tilt_angle = 0
pan_angle = 0
lock1 = 0

# Function to convert angle to pulse width
def angle_to_pulse(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    pulse = int(servo_min + (angle / 180.0) * (servo_max - servo_min))
    return pulse

# Set frequency to 60Hz, good for servos
pwm.set_pwm_freq(60)

def led_show_status():
    global status
    while True:
        r, g, b = 0, 0, 0
        s = 0  # Speed
        n = 0  # Number of flashes
        on = True
        fade = True  # Whether to use fading or just blinking

        # Determine color, speed, number of flashes, and fading based on the status
        if status == 1:  # Moving operation (fading)
            r, g, b = 255, 136, 0
            s, n, on, fade = 70, 3, True, False
        elif status == 2:  # Standby (fading)
            r, g, b = 0, 255, 0
            s, n, on, fade = 50, 1, True, False
        elif status == 3:  # Error (no fading)
            r, g, b = 255, 0, 0
            s, n, on, fade = 200, 3, False, False

        # Set the pixel color
        pixels.fill((g, r, b))
        pixels.show()

        # Flashing loop
        for _ in range(n):
            if fade:
                # Fade up
                for i in range(150, 251, 5):
                    value = int(math.pow(2, i / 32.0))
                    pixels.brightness = value / 255.0
                    pixels.show()
                    time.sleep(s / 1000.0)

                # Fade down
                for i in range(250, 149, -5):
                    value = int(math.pow(2, i / 32.0))
                    pixels.brightness = value / 255.0
                    pixels.show()
                    time.sleep(s / 1000.0)
            else:
                # Blink without fading
                pixels.brightness = 1.0
                pixels.show()
                time.sleep(s / 1000.0)
                pixels.brightness = 0.0
                pixels.show()
                time.sleep(s / 1000.0)

        # Clear the pixel if it's not meant to stay on
        if not on:
            pixels.fill((0, 0, 0))
            pixels.show()

        time.sleep(1)

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def scale_speed(speed):
    clamped_speed = clamp(speed, -1, 1)
    return int(clamped_speed * 100)

def drive_wheels(left_speed, right_speed):
    if left_speed < 0:
        left_speed = abs(left_speed)
        MotorLeft.backward(left_speed)
    elif left_speed == 0:
        MotorLeft.stop(0)
    else:
        MotorLeft.forward(left_speed)

    if right_speed < 0:
        right_speed = abs(right_speed)
        MotorRight.backward(right_speed)
    elif right_speed == 0:
        MotorRight.stop(0)
    else:
        MotorRight.forward(right_speed)


def move_robot(linear_velocity, angular_velocity):
    global left_wheel_speed, right_wheel_speed

    wheelbase = 0.5

    left_wheel_speed = linear_velocity - (angular_velocity * wheelbase / 2.0)
    right_wheel_speed = linear_velocity + (angular_velocity * wheelbase / 2.0)

    #print(linear_velocity, " ", angular_velocity)
    #left_wheel_speed = linear_velocity - (angular_velocity * wheelbase / 2.0)
    #right_wheel_speed = linear_velocity + (angular_velocity * wheelbase / 2.0)

    #needs to be changed
    scaled_left_speed = scale_speed(left_wheel_speed)
    scaled_right_speed = scale_speed(right_wheel_speed)

    drive_wheels(scaled_left_speed, scaled_right_speed)


def get_joystick_input(right_x, right_y):
    global movement_detected, last_movement_time

    right_joystick_x = right_x
    right_joystick_y = right_y

    max_linear_velocity = 1.0
    max_angular_velocity = 1.0

    linear_velocity = max_linear_velocity * right_joystick_y
    angular_velocity = max_angular_velocity * right_joystick_x

    if linear_velocity != 0 or angular_velocity != 0:
        movement_detected = True
        last_movement_time = time.time()
    else:
        movement_detected = False

    move_robot(linear_velocity, angular_velocity)


def convert_to_angle(value):
    """Convert the input value from [-1, 1] to an angle between 0 and 180 degrees."""
    angle = 90 + (value * 90)
    return angle


def update_angle():
    """Loop to continuously update the angle based on the input value."""
    while True:
        global pan1, tilt1, tilt_angle, pan_angle

        #print(pan1, '-', tilt1)

        # if tilt1 < -0.05 and tilt_angle >= -1:
        #     tilt_angle -= abs(tilt1)/5
        # elif tilt1 > 0.05  and tilt_angle <= 1:
        #     tilt_angle += abs(tilt1)/5
        
        # if pan1 < -0.05 and pan_angle >= -1:
        #     pan_angle -= abs(pan1)/5
        # elif pan1 > 0.05  and pan_angle <= 1:
        #     pan_angle += abs(pan1)/5
        
        # if pan1 > -0.3 and pan1 < 0.3:
        #     pan1 = 0

        # Clamp the input value to the range [-1, 1]
        print(-pan1)
        pane = max(-1, min(1, -pan1))
        tilte = max(-0.5, min(0.51, tilt1-0.2))
        # Calculate the current angle based on the value

        angle1 = convert_to_angle(pane)
        angle2 = convert_to_angle(tilte)

        pulse_width_pan = angle_to_pulse(angle1)
        pulse_width_tilt = angle_to_pulse(angle2)
        #print(pulse_width_tilt, ' - ', pulse_width_pan)
        if (not lock1):
            pwm.set_pwm(0, 0, pulse_width_pan)
        pwm.set_pwm(1, 0, pulse_width_tilt)
        time.sleep(0.1)




async def handle_connection(websocket, path):
    global value

    global pan1
    global tilt1
    global lock1
    print("Client connected")
    try:
        async for message in websocket:
            data = json.loads(message)

            buttons = data.get('buttons', {})

            lock = buttons.get('button4',{})
            
            axes = data.get('axes', {})

            angle = axes.get('axis2', 0)
            linear = axes.get('axis3', 0)

            pan = axes.get('axis0', 0)
            tilt = axes.get('axis1', 0)

            deadzone = 0.1

            if abs(angle) < deadzone:
                angle = 0
            if abs(linear) < deadzone:
                linear = 0

            if abs(pan) < deadzone:
                pan = 0
            if abs(tilt) < deadzone:
                tilt = 0


            # update_angle(pan, tilt)

            pan1 = pan
            tilt1 = tilt
            lock1 = lock


            get_joystick_input(angle, linear)

    except websockets.ConnectionClosed:
        print("Client disconnected")

async def main():
    async with websockets.serve(handle_connection, "0.0.0.0", 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()

def check_movement_timeout():
    global status, movement_detected
    while True:
        if movement_detected:
            status = 1

        else:
            if time.time() - last_movement_time > 1:
                movement_detected = False
                status = 2
        time.sleep(0.1)


# Start the LED thread
led_thread = threading.Thread(target=led_show_status)
led_thread.daemon = True
led_thread.start()

# Start the movement check thread
movement_thread = threading.Thread(target=check_movement_timeout)
movement_thread.daemon = True
movement_thread.start()

angle_thread = threading.Thread(target=update_angle)
angle_thread.daemon = True
angle_thread.start()


# Start the WebSocket server
if __name__ == "__main__":
    asyncio.run(main())