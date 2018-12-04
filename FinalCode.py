# Import necessary libraries
import RPi.GPIO as GPIO
import time

# Set GPIO Pins for the ultrasonic sensor
GPIO_TRIGGER = 18
GPIO_ECHO = 24
# Set GPIO Pins for the servos
RIGHT_MOTOR = 22
LEFT_MOTOR = 23

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(RIGHT_MOTOR, GPIO.OUT)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)

# Set PWN for servo pins at 100Hz
pwm = GPIO.PWM(RIGHT_MOTOR, 100)
pwm1 = GPIO.PWM(LEFT_MOTOR, 100)
# Start with 5 duty cycles
pwm.start(5)
pwm1.start(5)

# Update the duty cycle of the right servo given angle
def updateR(angle):
    duty = float(angle) / 10.0 + 2.5
    pwm.ChangeDutyCycle(duty)

# Update the duty cycle of the left servo given angle
def updateL(angle):
    duty = float(angle) / 10.0 + 2.5
    pwm1.ChangeDutyCycle(duty)

# Read the distance from the nearest object to the robot
# and perform object avoidance if necessary 
def distance():
    # Set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # Set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # Save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # Save arrival time
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # Calculate time difference
    TimeElapsed = StopTime - StartTime
    # Multiply elapsed time with the sonic speed (34300 cm/s)
    # Divide answer by 2 since time measured distance there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    try:
		# Infinitely loop
        while True:
            dist = distance()
            print ("Measured Distance = %.1f cm" % dist)

            # Back up and turn left if object within 40 cm of robot's vision
            if(dist < 40):
                # Backward
                updateR(130)
                updateL(50)
                time.sleep(1)
                # Turn left
                updateR(50)
                updateL(50)
                time.sleep(.5)

            # Forward
            updateR(50)
            updateL(130)
            time.sleep(1)
 
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.output(RIGHT_MOTOR, False)
        GPIO.output(LEFT_MOTOR, False)
        pwm.ChangeDutyCycle(0)
        pwm1.ChangeDutyCycle(0) 
        pwm.stop()
        pwm1.stop()
        GPIO.cleanup()
