import smbus
import math
import time
import RPi.GPIO as GPIO

# Define GPIO pins for servos
ROLL_SERVO_PIN = 18
PITCH_SERVO_PIN = 4

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ROLL_SERVO_PIN, GPIO.OUT)
GPIO.setup(PITCH_SERVO_PIN, GPIO.OUT)

# Initialize PWM for servos
roll_servo = GPIO.PWM(ROLL_SERVO_PIN, 50) # 50 Hz frequency
pitch_servo = GPIO.PWM(PITCH_SERVO_PIN, 50) # 50 Hz frequency
roll_servo.start(7.2) #7.2
pitch_servo.start(3.5) #3.5

# MPU6050 I2C address
MPU = 0x68

# Register addresses
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# MPU6050 sensitivities
ACC_SENSITIVITY = 16384.0  # +/- 2g range
GYRO_SENSITIVITY = 131.0    # +/- 250deg/s range

# Variables for data storage
AccX, AccY, AccZ = 0.0, 0.0, 0.0
GyroX, GyroY, GyroZ = 0.0, 0.0, 0.0
accAngleX, accAngleY = 0.0, 0.0
gyroAngleX, gyroAngleY, gyroAngleZ = 0.0, 0.0, 0.0
roll, pitch, yaw = 0.0, 0.0, 0.0

# Variables for error correction
AccErrorX, AccErrorY = 0.0, 0.0
GyroErrorX, GyroErrorY, GyroErrorZ = 0.0, 0.0, 0.0

# Set the deadband width, proportional gains, and reset interval
DEADBAND_WIDTH = 0.5  # Adjust as needed
PITCH_GAIN = 0.05  # Adjust as needed
ROLL_GAIN = 0.05  # Adjust as needed
RESET_INTERVAL = 5  # Reset gyroAngleX and gyroAngleY every 5 seconds
ANGULAR_VELOCITY_THRESHOLD = 0.5  # Adjust as needed

# Initialize variables for gyroAngle reset and angular velocity tracking
last_reset_time = time.time()
last_angular_velocity = 0.0

# Initialize the I2C bus
bus = smbus.SMBus(1)

def read_word_2c(addr):
    high = bus.read_byte_data(MPU, addr)
    low = bus.read_byte_data(MPU, addr+1)
    val = (high << 8) + low
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def setup_MPU():
    bus.write_byte_data(MPU, 0x6B, 0x00) # Reset
    time.sleep(0.1)

def calculate_IMU_error():
    global AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ
    c = 0
    while c < 200:
        denominator = math.sqrt(AccX**2 + AccZ**2)
        if denominator != 0:  # Check for division by zero
            AccErrorX += math.atan(AccY / denominator) * 180 / math.pi
        c += 1
    AccErrorX /= 200

    c = 0
    while c < 200:
        denominator = math.sqrt(AccY**2 + AccZ**2)
        if denominator != 0:  # Check for division by zero
            AccErrorY += math.atan(-1 * (AccX) / denominator) * 180 / math.pi
        c += 1
    AccErrorY /= 200

    c = 0
    while c < 200:
        GyroErrorX += read_word_2c(GYRO_XOUT_H) / GYRO_SENSITIVITY
        GyroErrorY += read_word_2c(GYRO_XOUT_H + 2) / GYRO_SENSITIVITY
        GyroErrorZ += read_word_2c(GYRO_XOUT_H + 4) / GYRO_SENSITIVITY
        c += 1
    GyroErrorX /= 200
    GyroErrorY /= 200
    GyroErrorZ /= 200

    print("AccErrorX:", AccErrorX)
    print("AccErrorY:", AccErrorY)
    print("GyroErrorX:", GyroErrorX)
    print("GyroErrorY:", GyroErrorY)
    print("GyroErrorZ:", GyroErrorZ)

def read_IMU_data():
    global AccX, AccY, AccZ, GyroX, GyroY, GyroZ
    AccX = read_word_2c(ACCEL_XOUT_H) / ACC_SENSITIVITY
    AccY = read_word_2c(ACCEL_XOUT_H + 2) / ACC_SENSITIVITY
    AccZ = read_word_2c(ACCEL_XOUT_H + 4) / ACC_SENSITIVITY

    GyroX = read_word_2c(GYRO_XOUT_H) / GYRO_SENSITIVITY
    GyroY = read_word_2c(GYRO_XOUT_H + 2) / GYRO_SENSITIVITY
    GyroZ = read_word_2c(GYRO_XOUT_H + 4) / GYRO_SENSITIVITY

def complementary_filter():
    global roll, pitch, yaw, gyroAngleX, gyroAngleY, gyroAngleZ, last_reset_time, last_angular_velocity
    if AccX != 0 or AccZ != 0:  
        accAngleX = math.atan(AccY / math.sqrt(AccX**2 + AccZ**2)) * 180 / math.pi - 0.58
    else:
        accAngleX = 0
    if AccY != 0 or AccZ != 0:  
        accAngleY = math.atan(-1 * AccX / math.sqrt(AccY**2 + AccZ**2)) * 180 / math.pi + 1.58
    else:
        accAngleY = 0

    # Calculate elapsed time
    current_time = time.time()
    elapsed_time_since_reset = current_time - last_reset_time

    # Reset gyroAngleX and gyroAngleY periodically
    #if elapsed_time_since_reset >= RESET_INTERVAL:
     #   gyroAngleX = 3.5
    #    gyroAngleY = 7.2
   #     last_reset_time = current_time

    gyroAngleX += GyroX * elapsedTime
    gyroAngleY += GyroY * elapsedTime
    yaw += GyroZ * elapsedTime
    # Calculate angular velocity
    angular_velocity = math.sqrt(GyroX**2 + GyroY**2 + GyroZ**2)

    # If angular velocity is below threshold, consider the board stationary and keep pitch and roll values constant
    if angular_velocity < ANGULAR_VELOCITY_THRESHOLD:
        pitch = last_angular_velocity if abs(last_angular_velocity) < DEADBAND_WIDTH else 0.0
        roll = pitch
    else:
        # Calculate pitch and roll using complementary filter with different gains
        gyroAngleX = 0.90 * gyroAngleX + 0.04 * accAngleX
        gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY
        roll = gyroAngleX
        pitch = gyroAngleY
        #time.sleep(100)
        

    # Store current angular velocity for next iteration
    last_angular_velocity = angular_velocity

    # Apply deadband to roll and pitch
    if abs(roll) < DEADBAND_WIDTH:
        roll = 0.0
    if abs(pitch) < DEADBAND_WIDTH:
        pitch = 0.0
        
    # Limit maximum roll value
    #MAX_ROLL_ANGLE = 10.0  # Adjust as needed
    #if (roll) > MAX_ROLL_ANGLE:
    #    roll = 0

    # Apply proportional control to stabilize the servos with different gains
    roll_duty_cycle = 7.5 + ROLL_GAIN * roll
    pitch_duty_cycle = 7.5 + PITCH_GAIN * pitch

    # Limit duty cycle to avoid extreme positions
    roll_duty_cycle = max(5.5, min(roll_duty_cycle, 8.2))
    pitch_duty_cycle = max(2.5, min(pitch_duty_cycle, 6.0))

    # Set servo positions
    roll_servo.ChangeDutyCycle(roll_duty_cycle)
    #pitch_servo.ChangeDutyCycle(pitch_duty_cycle)

# Initialize MPU6050
setup_MPU()
calculate_IMU_error()

# Initialize gyro angle variables
gyroAngleX, gyroAngleY, gyroAngleZ = 3.5, 7.2, 0.0

# Variables for timing
previousTime = time.time()

while True:
    # Calculate elapsed time
    currentTime = time.time()
    elapsedTime = currentTime - previousTime
    previousTime = currentTime

    read_IMU_data()
    complementary_filter()
    print("Roll:", roll)
    print("Pitch:", pitch)
    time.sleep(0.2)  # Add a small delay to avoid excessive computation and servo jitter

