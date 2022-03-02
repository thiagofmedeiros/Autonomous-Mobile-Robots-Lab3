"""lab2_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

WHEEL_DIST = 1.05
# it seems that the wheel diameter is half of what said in lab 1
WHEEL_DIAMETER = 1.6
MAX_PHI = 6.28
MAX_SIMULATION_TIME = 30 * 1000
MAX_MEASURED_DISTANCE = 1.27
ACCEPTED_ERROR = 0.001

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


def move(K):
    global time

    while time/1000 < 30:
        front, left, right, yaw = getSensors()

        DESIRED_DISTANCE = (2.5 + 5.5) / 2

        rightDistance = right * abs(math.cos(getYawRadians()))
        leftDistance = left * abs(math.cos(getYawRadians()))

        Rrps = K * (DESIRED_DISTANCE - rightDistance)
        Lrps = K * (DESIRED_DISTANCE - leftDistance)

        setSpeedsRPS(Lrps, Rrps)

        robot.step(timestep)
        time += timestep
        print("Time {0:.3f} seconds\n".format(time / 1000))

        setSpeedsRPS(0, 0)
        print("\nSimulation Stopped\n")


time = 0
setSpeedsRPS(0, 0)

robot.step(timestep)
time += timestep

move(90)
