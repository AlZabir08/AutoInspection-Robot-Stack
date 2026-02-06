import math

def compute_motor_speeds(current_pos, current_yaw, target_pos, max_speed=6.0):
    x, y = current_pos[0], current_pos[1]
    tx, ty = target_pos

    dx = tx - x
    dy = ty - y
    target_angle = math.atan2(dy, dx)
    heading_error = target_angle - current_yaw
    heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

    Kp = 5.0
    turn = Kp * heading_error

    left_speed = max_speed - turn
    right_speed = max_speed + turn

    left_speed = max(-max_speed, min(left_speed, max_speed))
    right_speed = max(-max_speed, min(right_speed, max_speed))

    return left_speed, right_speed
