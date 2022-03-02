"""lab2_task3corridor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

WHEEL_DIST = 1.05
# it seems that the wheel diameter is half of what said in lab 1
WHEEL_DIAMETER = 1.6
MAX_PHI = 6.28
MAX_SIMULATION_TIME = 30 * 1000
MAX_MEASURED_DISTANCE = 50
ACCEPTED_ERROR = 0.001
K1 = 0.1
K2 = 0.5
K3 = 1
K4 = 2
K5 = 2.5
K6 = 5
DESIRED_DISTANCE = 8

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# initialization of motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# distance sensors
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)


def setSpeedsRPS(rpsLeft, rpsRight):
    if rpsLeft > MAX_PHI:
        print("Saturating left speed to {0:.3f} rad/s".format(MAX_PHI))
        leftMotor.setVelocity(MAX_PHI)
    elif rpsLeft < -MAX_PHI:
        print("Saturating left speed to {0:.3f} rad/s".format(-MAX_PHI))
        leftMotor.setVelocity(-MAX_PHI)
    else:
        print("Left motor velocity: {0:.3f} rad/s".format(rpsLeft))
        leftMotor.setVelocity(rpsLeft)

    if rpsRight > MAX_PHI:
        print("Saturating right speed to {0:.3f} rad/s".format(MAX_PHI))
        rightMotor.setVelocity(MAX_PHI)
    elif rpsRight < -MAX_PHI:
        print("Saturating right speed to {0:.3f} rad/s".format(-MAX_PHI))
        rightMotor.setVelocity(-MAX_PHI)
    else:
        print("Right motor velocity: {0:.3f} rad/s\n".format(rpsRight))
        rightMotor.setVelocity(rpsRight)


def inchesToMeters(x):
    return x / 39.37


def metersToInches(x):
    return x * 39.37


def getSensors():
    fdsVal = metersToInches(frontDistanceSensor.getValue())
    ldsVal = metersToInches(leftDistanceSensor.getValue())
    rdsVal = metersToInches(rightDistanceSensor.getValue())
    yaw = getYawDegrees()

    print("FrontDist {0:.3f} inches".format(fdsVal))
    print("LeftDist {0:.3f} inches".format(ldsVal))
    print("RightDist {0:.3f} inches\n".format(rdsVal))
    print("Yaw = {0:.2f}".format(getYawDegrees()))

    return fdsVal, ldsVal, rdsVal, yaw


def getYawDegrees():
    Yaw = math.degrees(getYawRadians())

    if Yaw < 0:
        Yaw = Yaw + 360

    return Yaw


def getYawRadians():
    return imu.getRollPitchYaw()[2]


def rotateUntilFindWall(K):
    global time

    front, left, right, yaw = getSensors()

    while right > 8:
        # turn left
        rps = K * ((DESIRED_DISTANCE - right) / (timestep / 1000))

        setSpeedsRPS(rps, -rps)

        robot.step(timestep)
        time += timestep
        print("Time {0:.3f} seconds\n".format(time / 1000))

        front, left, right, yaw = getSensors()


def turn90(K):
    global time

    yaw = getYawDegrees()

    desiredYaw = yaw + 90

    if desiredYaw > 360:
        desiredYaw -= 360

    while abs(getYawDegrees() - desiredYaw) > 5:
        setSpeedsRPS(-MAX_PHI, MAX_PHI)

        robot.step(timestep)
        time += timestep
        print("Time {0:.3f} seconds\n".format(time / 1000))


def move(K):
    global time

    rotateUntilFindWall(K)

    while time / 1000 < 3 * 60:
        front, left, right, yaw = getSensors()

        if front < 3:
            turn90(K)

        if 315 < yaw < 360 or 0 < yaw <= 45:
            right = right * math.cos(getYawRadians())
            left = left * math.cos(getYawRadians())

        elif 45 < yaw < 135:
            right = right * math.sin(getYawRadians())
            left = left * math.sin(getYawRadians())

        elif 135 < yaw < 225:
            right = right * math.cos(getYawRadians()) * (-1)
            left = left * math.cos(getYawRadians()) * (-1)

        elif 225 < yaw < 315:
            right = right * math.sin(getYawRadians()) * (-1)
            left = left * math.sin(getYawRadians()) * (-1)

        print("rightDistance: {0:.3f}".format(right))
        print("leftDistance: {0:.3f}".format(left))

        Rrps = K * (DESIRED_DISTANCE - right) / (timestep / 1000)
        Lrps = K * (DESIRED_DISTANCE - left) / (timestep / 1000)

        # Keep distance from wall
        if right < 3:
            setSpeedsRPS(-Rrps, Rrps)
            robot.step(timestep)
            time += timestep
            print("Time {0:.3f} seconds\n".format(time / 1000))
        elif right > 5:
            setSpeedsRPS(Rrps, -Rrps)
            robot.step(timestep)
            time += timestep
            print("Time {0:.3f} seconds\n".format(time / 1000))

        # Step forward for not rotating infinitely
        setSpeedsRPS(MAX_PHI, MAX_PHI)
        robot.step(timestep)
        time += timestep
        print("Time {0:.3f} seconds\n".format(time / 1000))


time = 0
setSpeedsRPS(0, 0)

robot.step(timestep)
time += timestep

move(K1)

setSpeedsRPS(0, 0)
print("\nSimulation Stopped\n")