#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# =============================================================================
# 1. 초기화 및 설정
# =============================================================================
ev3 = EV3Brick()

# 포트 설정
grab_motor = Motor(Port.B)
left_motor = Motor(Port.C)
right_motor = Motor(Port.A)

left_sensor = ColorSensor(Port.S1)       # 왼쪽 센서
right_sensor = ColorSensor(Port.S4)      # 오른쪽 센서
object_detector = ColorSensor(Port.S3)   # 정면 기둥 감지
ultra_sensor = UltrasonicSensor(Port.S2) # 거리 감지

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=114)
robot.settings(straight_speed=100, straight_acceleration=100, turn_rate=100, turn_acceleration=100)

# 상수
N, E, S, W = 0, 1, 2, 3
KP = 1.2
BASE_SPEED = 100
GRAB_DIST = 50      
BLACK_LIMIT = 20
TARGET_VAL = 50

# 상태 변수
carrying_pillar = False
current_x, current_y = 0, 0
current_dir = N

# =============================================================================
# 2. 기본 함수
# =============================================================================

def gripper_init():
    grab_motor.run_until_stalled(-500, then=Stop.HOLD, duty_limit=50)

def gripper_close():
    grab_motor.run_until_stalled(500, then=Stop.HOLD, duty_limit=60)

def gripper_open():
    grab_motor.run_until_stalled(-500, then=Stop.HOLD, duty_limit=50)

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    if diff == 1: robot.turn(90)
    elif diff == 2: robot.turn(180)
    elif diff == 3: robot.turn(-90)
    return target_dir

# =============================================================================
# 3. 주행 로직
# =============================================================================

def follow_line_one_cell(moving_dir):
    global current_x, current_y
    robot.straight(60) 
    
    while True:
        if not carrying_pillar:
            if ultra_sensor.distance() < GRAB_DIST + 50:
                robot.stop()
                return "OBJECT"

        arrived = False
        error = 0

        # 트레이싱 방향 결정
        use_left_tracing = False
        if moving_dir == N or moving_dir == W: use_left_tracing = True
        elif moving_dir == E and current_y == 4: use_left_tracing = True
        elif moving_dir == S and current_x == 2: use_left_tracing = True

        if use_left_tracing:
            if right_sensor.reflection() < BLACK_LIMIT: arrived = True
            error = left_sensor.reflection() - TARGET_VAL
        else:
            if left_sensor.reflection() < BLACK_LIMIT: arrived = True
            error = TARGET_VAL - right_sensor.reflection()

        if arrived:
            robot.stop()
            return "ARRIVED"

        robot.drive(BASE_SPEED, error * KP)
        wait(10)

def move_manhattan(goal_x, goal_y):
    global current_x, current_y, current_dir
    dx = goal_x - current_x
    dy = goal_y - current_y
    
    if dx != 0:
        target_dir = E if dx > 0 else W
        current_dir = turn_min(current_dir, target_dir)
        for _ in range(abs(dx)):
            res = follow_line_one_cell(target_dir)
            current_x += 1 if target_dir == E else -1
            if res == "OBJECT": return "OBJECT_FOUND"

    if dy != 0:
        target_dir = N if dy > 0 else S
        current_dir = turn_min(current_dir, target_dir)
        for _ in range(abs(dy)):
            res = follow_line_one_cell(target_dir)
            current_y += 1 if target_dir == N else -1
            if res == "OBJECT": return "OBJECT_FOUND"
                
    return "ARRIVED"

def move_safe(target_x, target_y):
    global current_x, current_y
    
    if current_y == 0 and current_x != 0:
        res = move_manhattan(current_x, 1)
        if res == "OBJECT_FOUND": return res

    if target_y == 0:
        if current_x != 0:
            res = move_manhattan(0, current_y)
            if res == "OBJECT_FOUND": return res
        if current_y != 1:
            res = move_manhattan(0, 1)
            if res == "OBJECT_FOUND": return res
        res = move_manhattan(target_x, 1)
        if res == "OBJECT_FOUND": return res
        return move_manhattan(target_x, 0)

    else:
        is_up_now = (current_y >= 2)
        is_up_target = (target_y >= 2)
        
        if is_up_now == is_up_target:
            return move_manhattan(target_x, target_y)
        else:
            res = move_manhattan(0, current_y)
            if res == "OBJECT_FOUND": return res
            res = move_manhattan(0, target_y)
            if res == "OBJECT_FOUND": return res
            return move_manhattan(target_x, target_y)

# =============================================================================
# 4. 미션 수행 (수정됨: 일단 잡고 -> 색확인)
# =============================================================================

def handle_pillar():
    global carrying_pillar, current_dir
    
    # 1. 발견 및 접근
    ev3.speaker.beep()
    while ultra_sensor.distance() > GRAB_DIST:
        robot.drive(50, 0)
    robot.stop()
    
    # 2. [선 행동] 묻지도 따지지도 않고 일단 잡는다!
    # 집게 안으로 기둥을 넣기 위해 6cm 전진
    robot.straight(60) 
    gripper_close()
    carrying_pillar = True
    wait(500) # 잡고 나서 기둥이 안정될 때까지 잠깐 대기
    
    # 3. [후 판단] 잡은 상태에서 느긋하게 색상 확인
    detected_color = Color.BLACK
    
    # 5번 체크 (이제 거리가 고정되어서 정확도 높음)
    for i in range(5):
        c = object_detector.color()
        if c in [Color.RED, Color.YELLOW, Color.ORANGE]:
            detected_color = Color.RED
            ev3.speaker.beep(500)
            break
        elif c in [Color.BLUE, Color.GREEN, Color.CYAN]:
            detected_color = Color.BLUE
            ev3.speaker.beep(1000)
            break
        wait(50)

    # 4. 그래도 모르면? 빨강으로 간주 (Force Delivery)
    if detected_color == Color.BLACK:
        detected_color = Color.RED 
        ev3.speaker.beep(2000) # 고음: 강제 모드

    # 목표 설정
    tx, ty = -1, -1
    if detected_color == Color.RED: tx, ty = 1, 0  
    elif detected_color == Color.BLUE: tx, ty = 2, 0 

    saved_x, saved_y = current_x, current_y
    
    # 5. 정렬 전진 (이미 6cm 왔으므로, 회전축 맞추기 위해 2cm만 더)
    robot.straight(20)
    
    # 6. 턴 & 배달
    current_dir = turn_min(current_dir, (current_dir + 2) % 4)
    move_safe(tx, ty)
    
    # 하차
    current_dir = turn_min(current_dir, S)
    gripper_open()
    robot.straight(-100)
    carrying_pillar = False
    
    move_safe(saved_x, saved_y)

# =============================================================================
# 5. BFS 알고리즘
# =============================================================================

class GridMap:
    def __init__(self):
        self.width = 3
        self.height = 5
        self.walls = [{(1, 1), (1, 2)}, {(2, 1), (2, 2)}]

    def is_connected(self, node_a, node_b):
        if node_a[1] == 0 and node_b[1] == 0: return False
        pair = {node_a, node_b}
        for wall in self.walls:
            if pair == wall: return False
        return True

    def get_neighbors(self, node):
        x, y = node
        neighbors = []
        candidates = []
        if x > 0: candidates.append((x - 1, y))
        if x < self.width - 1: candidates.append((x + 1, y))
        if y > 0: candidates.append((x, y - 1))
        if y < self.height - 1: candidates.append((x, y + 1))
        for next_node in candidates:
            if self.is_connected(node, next_node): neighbors.append(next_node)
        return neighbors

def bfs_shortest_path(grid, start, goal):
    queue = [[start]]
    visited = {start}
    if start == goal: return [start]
    while queue:
        path = queue.pop(0)
        node = path[-1]
        if node == goal: return path[1:]
        for neighbor in grid.get_neighbors(node):
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    return []

def generate_full_path(start_pos):
    grid = GridMap()
    targets = []
    for x in range(3):
        for y in range(1, 5):
            targets.append((x, y))
    visited_targets = set()
    if start_pos in targets: visited_targets.add(start_pos)
    full_path_plan = []
    current = start_pos
    while len(visited_targets) < len(targets):
        best_dist = 999
        best_target = None
        best_path = []
        for t in targets:
            if t not in visited_targets:
                path = bfs_shortest_path(grid, current, t)
                if len(path) > 0:
                    dist = len(path)
                    if dist < best_dist:
                        best_dist = dist
                        best_target = t
                        best_path = path
                    elif dist == best_dist:
                        if t[1] > best_target[1]:
                            best_target = t
                            best_path = path
                        elif t[1] == best_target[1] and abs(t[0] - current[0]) < abs(best_target[0] - current[0]):
                             best_target = t
                             best_path = path
        if best_target:
            full_path_plan.extend(best_path)
            visited_targets.add(best_target)
            current = best_target
        else: break
    return full_path_plan

# =============================================================================
# 6. 메인 실행
# =============================================================================

def main():
    gripper_init()
    
    print("Moving to (0, 2)...")
    target_init = (0, 2)
    while True:
        status = move_safe(target_init[0], target_init[1])
        if status == "OBJECT_FOUND":
            handle_pillar()
        else:
            break 

    print("Generating BFS Path...")
    waypoints = generate_full_path((0, 2))
    
    for wp in waypoints:
        while True:
            status = move_safe(wp[0], wp[1])
            if status == "OBJECT_FOUND":
                handle_pillar()
            else:
                break
    robot.stop()

if __name__ == '__main__':
    main()