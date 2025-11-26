#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# EV3 기본 설정
ev3 = EV3Brick()

grab_motor = Motor(Port.B)
left_motor = Motor(Port.C)
right_motor = Motor(Port.A)

left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, 56, 114)

N, E, S, W = 1, 2, 3, 4

threshold = 40
kp = 1.0

def follow_line_one_cell():
    robot.reset()
    while robot.distance() < 150:
        reflection = left_sensor.reflection()
        error = reflection - threshold
        robot.drive(120, kp * error)
        wait(10)
    robot.stop()

def grab_object():
    grab_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

def release_object():
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, 180, -90][diff]
    robot.turn(angle)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir):
    x, y = start_xy
    gx, gy = goal_xy

    dx = gx - x
    dy = gy - y

    if dx != 0:
        target = E if dx > 0 else W
        now_dir = turn_min(now_dir, target)
        for _ in range(abs(dx)):
            follow_line_one_cell()
            x += 1 if target == E else -1

    if dy != 0:
        target = N if dy > 0 else S
        now_dir = turn_min(now_dir, target)
        for _ in range(abs(dy)):
            follow_line_one_cell()
            y += 1 if target == N else -1

    return (x, y), now_dir

now_pos = (0, 0)
now_dir = N

(now_pos, now_dir) = move_manhattan(now_pos, (0, 2), now_dir)

while ultra_sensor.distance() > 35:
    robot.drive(50, 0)
robot.stop()
grab_object()

obj_color = object_detector.color()
ev3.speaker.beep()

if obj_color == Color.RED:
    goal = (-1, 0)
elif obj_color == Color.BLUE:
    goal = (-2, 0)
else:
    goal = (0, 1)

(now_pos, now_dir) = move_manhattan(now_pos, goal, now_dir)

release_object()

(now_pos, now_dir) = move_manhattan(now_pos, (0, 0), now_dir)

ev3.speaker.beep()