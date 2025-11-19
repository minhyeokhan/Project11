#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
left_motor = Motor(Port.C)
right_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, 55.5, 104)
cs1 = ColorSensor(Port.S1)
cs2 = ColorSensor(Port.S4)

threshold = 50
kp = 1.2

now_dir = 1
target_cor = [-2, 1]

ev3.speaker.beep()

for i in range(abs(target_cor[1])):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection > 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

if target_cor[0] > 0:
    target_dir = 2
    
elif target_cor[0] < 0:
    target_dir = 4

    
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

now_dir = target_dir

for i in range(abs(target_cor[0])):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection > 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

if target_cor[0] > 0:
    target_dir = 3
elif target_cor[0] < 0:
    target_dir = 1

direction = abs((target_dir - now_dir) % 4)
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

now_dir = target_dir

target_cor = [2,3]

for i in range(abs(target_cor[1])):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if right_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if right_reflection > 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

if target_cor[0] > 0:
    target_dir = 2
elif target_cor[0] < 0:
    target_dir = 4
    
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

now_dir = target_dir

for i in range(abs(target_cor[0])):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if right_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if right_reflection > 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if right_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

if target_cor[0] > 0:
    target_dir = 1
elif target_cor[0] < 0:
    target_dir = 3

direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

now_dir = target_dir

target_cor = [0,1]

for i in range(abs(target_cor[1])):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection > 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

if target_cor[0] > 0:
    target_dir = 1
elif target_cor[0] < 0:
    target_dir = 3
    
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)

now_dir = target_dir

for i in range(abs(target_cor[0])):
    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection < 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)

    while True:
        left_reflection = cs1.reflection()
        right_reflection = cs2.reflection()
        if left_reflection > 30:
            ev3.speaker.beep()
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
    wait(10)