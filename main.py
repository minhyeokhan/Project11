#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# --- 1. 설정 및 초기화 ---
ev3 = EV3Brick()

# 모터 및 센서
grab_motor = Motor(Port.B)
left_motor = Motor(Port.C)
right_motor = Motor(Port.A)

left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S2)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)

# --- 2. 상수 및 좌표 정의 ---
N, E, S, W = 1, 2, 3, 4
THRESHOLD = 45  # Script A 값 적용 (안정적)
KP = 0.7        # Script A 값 적용 (안정적)

# 거리 관련 상수
DETECT_DIST = 250
APPROACH_FAST_DIST = 150
APPROACH_SLOW_DIST = 30 # Script A의 3cm 정지 로직
MAX_OFFLINE = 300

# 좌표 정의
START = (0, 2)
RED_HOME = (0, 1)  # 빨간색 놓는 곳
BLUE_HOME = (0, 0) # 파란색 놓는 곳 (기존 코드 기준 수정, 필요시 변경)

# 코너 좌표 매핑
corner_pos = {
    1: (2, 2), 2: (3, 2), 3: (4, 2),
    5: (2, 1), 6: (3, 1), 7: (4, 1),
    9: (2, 0), 10: (3, 0), 11: (4, 0),
}

# 특수 코너 (라인에서 벗어나야 하는 곳)
# 형식: {인덱스: (기준좌표 인덱스, 바라볼 방향)}
special_corners = {
    4: (3, E),
    8: (7, E),
    12: (11, E),
}

# 방문 순서 (사용자 정의)
corner_order = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

# --- 3. 기본 동작 함수 ---

def grab_object():
    """집게 닫기"""
    grab_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

def release_object():
    """집게 벌리기"""
    grab_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)

# --- 4. 라인 트레이싱 및 이동 (Script A 로직 이식) ---

def follow_line(speed, kp, mode="left_trace"):
    """
    mode="left_trace": 왼쪽 센서 사용 (갈 때)
    mode="right_trace": 오른쪽 센서 사용 (올 때/뒤집혔을 때)
    """
    if mode == "left_trace":
        error = left_sensor.reflection() - THRESHOLD
        turn_rate = kp * error
        robot.drive(speed, turn_rate)
    elif mode == "right_trace":
        error = right_sensor.reflection() - THRESHOLD
        turn_rate = -kp * error 
        robot.drive(speed, turn_rate)

def n_move(n, direction="go"):
    """
    direction="go": 갈 때 (왼쪽 트레이싱, 오른쪽 교차로 감지)
    direction="back": 올 때 (오른쪽 트레이싱, 왼쪽 교차로 감지)
    """
    for _ in range(n):
        if direction == "go":
            while right_sensor.reflection() > THRESHOLD:
                follow_line(100, 1.2, mode="left_trace")
            while right_sensor.reflection() <= THRESHOLD:
                follow_line(100, 1.2, mode="left_trace")
        elif direction == "back":
            while left_sensor.reflection() > THRESHOLD:
                follow_line(100, 1.2, mode="right_trace")
            while left_sensor.reflection() <= THRESHOLD:
                follow_line(100, 1.2, mode="right_trace")
    robot.stop()

def turn_min(now_dir, target_dir):
    """최단 회전 계산"""
    if now_dir == target_dir: return now_dir
    diff = target_dir - now_dir
    # 방향 차이 계산
    if diff == 1 or diff == -3: robot.turn(90)
    elif diff == -1 or diff == 3: robot.turn(-90)
    elif diff == 2 or diff == -2: robot.turn(180)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir, move_mode="go"):
    """
    좌표 이동 함수
    move_mode="go" -> 일반 주행 (왼쪽 센서 트레이싱)
    move_mode="back" -> 180도 회전 후 주행 (오른쪽 센서 트레이싱)
    """
    x, y = start_xy
    gx, gy = goal_xy
    dx = gx - x
    dy = gy - y
    
    # X축 이동
    if dx != 0:
        target_dir = E if dx > 0 else W
        now_dir = turn_min(now_dir, target_dir)
        n_move(abs(dx), direction=move_mode)
        x = gx

    # Y축 이동
    if dy != 0:
        target_dir = N if dy > 0 else S
        now_dir = turn_min(now_dir, target_dir)
        n_move(abs(dy), direction=move_mode)
        y = gy

    robot.stop()
    return (x, y), now_dir

# --- 5. 접근 및 잡기 로직 (Script A + B 통합) ---

def approach_and_grab_on_line():
    """라인 위 물체 접근 (2단계 감속 적용)"""
    # 1. 감지 안되면 패스
    if ultra_sensor.distance() > DETECT_DIST:
        return None

    # 2. 빠른 접근
    while ultra_sensor.distance() > APPROACH_FAST_DIST:
        robot.drive(150, 0)
    
    # 3. 정밀 접근 (3cm까지) - Script A 핵심
    while ultra_sensor.distance() > APPROACH_SLOW_DIST:
        robot.drive(30, 0)
    
    robot.stop()
    grab_object()
    wait(500)
    
    c = object_detector.color()
    ev3.speaker.beep()
    return c

def approach_and_grab_offline():
    """라인 밖 물체 접근 (거리 측정 후 복귀)"""
    robot.reset()
    # 접근
    while ultra_sensor.distance() > APPROACH_SLOW_DIST and robot.distance() < MAX_OFFLINE:
        robot.drive(80, 0)
    robot.stop()
    
    dist_traveled = robot.distance()
    
    # 물체가 너무 멀리 있어서 못 잡은 경우
    if ultra_sensor.distance() > APPROACH_SLOW_DIST + 50: 
        return None, dist_traveled
        
    grab_object()
    wait(500)
    c = object_detector.color()
    ev3.speaker.beep()
    
    return c, dist_traveled

# --- 6. 코너 처리 핸들러 (메인 로직) ---

def handle_normal_corner(idx, now_pos, now_dir):
    """일반 코너 작업 수행"""
    target_pos = corner_pos[idx]
    
    # 1. 목표 지점으로 이동 (갈 때 모드)
    now_pos, now_dir = move_manhattan(now_pos, target_pos, now_dir, move_mode="go")
    
    # 2. 물체 확인 및 잡기
    color = approach_and_grab_on_line()
    
    if color is None:
        # 물체 없으면 현재 상태 그대로 리턴
        return now_pos, now_dir

    # 3. 목표 지점 설정
    if color == Color.RED:
        home_goal = RED_HOME
    elif color == Color.BLUE:
        home_goal = BLUE_HOME
    else:
        # 색상 모르면 일단 놓기 (혹은 특정 구역)
        release_object()
        return now_pos, now_dir

    # -------------------------------------------------------
    # 4. [Script A 핵심] 잡은 후 복귀 시퀀스
    # -------------------------------------------------------
    
    # (1) 4cm 전진
    robot.straight(40)
    
    # (2) 180도 회전
    robot.turn(180)
    
    # (3) 방향 변수 반전 (N<->S, E<->W)
    if now_dir == N: now_dir = S
    elif now_dir == S: now_dir = N
    elif now_dir == E: now_dir = W
    elif now_dir == W: now_dir = E
    
    # 5. 홈으로 복귀 (올 때 모드: move_mode="back")
    now_pos, now_dir = move_manhattan(now_pos, home_goal, now_dir, move_mode="back")
    
    # 6. 물체 놓기
    release_object()
    
    # -------------------------------------------------------
    # 7. [중요] 다음 코너를 가기 위해 다시 정면 보기
    # -------------------------------------------------------
    # 홈에 도착해서 물체를 놓고 나면 벽을 보고 있거나 뒤집혀 있음
    # 다음 루프에서 move_mode="go"를 쓰려면 다시 180도 돌려놔야 함
    
    robot.straight(-40) # 물체와 거리 두기
    robot.turn(180)     # 다시 돌기
    
    # 방향 변수 다시 반전 (원상복구)
    if now_dir == N: now_dir = S
    elif now_dir == S: now_dir = N
    elif now_dir == E: now_dir = W
    elif now_dir == W: now_dir = E
    
    return now_pos, now_dir

def handle_special_corner(idx, now_pos, now_dir):
    """특수 코너 (라인 밖) 작업 수행"""
    base_idx, leave_dir = special_corners[idx]
    base_pos = corner_pos[base_idx]
    
    # 1. 기준 좌표로 이동
    now_pos, now_dir = move_manhattan(now_pos, base_pos, now_dir, move_mode="go")
    
    # 2. 바라봐야 할 방향으로 회전
    now_dir = turn_min(now_dir, leave_dir)
    
    # 3. 오프라인 접근 및 잡기
    color, dist = approach_and_grab_offline()
    
    # 4. 원래 위치로 후진 복귀
    robot.straight(-dist)
    
    if color is None:
        return now_pos, now_dir

    # 5. 목표 설정
    if color == Color.RED:
        home_goal = RED_HOME
    elif color == Color.BLUE:
        home_goal = BLUE_HOME
    else:
        release_object()
        return now_pos, now_dir

    # 6. 복귀 시퀀스 (특수 코너는 이미 정면을 보고 후진해왔으므로 180도 회전 필요함)
    # 왜냐하면 '복귀'는 move_mode="back" (오른쪽 센서)를 쓰기로 통일했기 때문
    
    robot.straight(40) # 약간 전진
    robot.turn(180)    # 뒤로 돌기
    
    if now_dir == N: now_dir = S
    elif now_dir == S: now_dir = N
    elif now_dir == E: now_dir = W
    elif now_dir == W: now_dir = E
    
    # 7. 홈 이동
    now_pos, now_dir = move_manhattan(now_pos, home_goal, now_dir, move_mode="back")
    
    # 8. 놓기 및 재정렬
    release_object()
    
    robot.straight(-40)
    robot.turn(180)
    
    if now_dir == N: now_dir = S
    elif now_dir == S: now_dir = N
    elif now_dir == E: now_dir = W
    elif now_dir == W: now_dir = E
    
    return now_pos, now_dir

# ----------------------------------------------------
# --- 7. 메인 실행 루프 ---
# ----------------------------------------------------

ev3.speaker.beep()
release_object()

now_pos = START
now_dir = N

# 시작 지점 탈출 (옵션)
# 만약 START가 (0,2)이고 로봇이 박스 안에 있다면 처음에만 살짝 빼줌
robot.straight(100)
n_move(1, direction="go") 
# 이제 로봇은 (0,2) 교차로를 막 지난 상태라고 가정하거나, 
# START 좌표를 로봇의 실제 시작 위치로 잘 맞춰야 함.
# 여기서는 START=(0,2)에서 시작한다고 가정.

for idx in corner_order:
    ev3.screen.print("Target: " + str(idx))
    
    if idx in special_corners:
        now_pos, now_dir = handle_special_corner(idx, now_pos, now_dir)
    elif idx in corner_pos:
        now_pos, now_dir = handle_normal_corner(idx, now_pos, now_dir)
        
    # 루프 하나가 끝나면 로봇은 Home에서 다시 필드를 보고 있는 상태 (move_mode="go" 준비 완료)

# 모든 작업 완료 후 시작 지점으로 복귀 (선택)
# now_pos, now_dir = move_manhattan(now_pos, START, now_dir, move_mode="go")
ev3.speaker.beep()