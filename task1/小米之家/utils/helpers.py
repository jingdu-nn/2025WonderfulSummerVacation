import math
import configparser
import os

def read_conf(path="configs.ini"):
    config = configparser.ConfigParser()
    default_conf = {
        'a_wh': 'A-1',
        'b_wh': 'B-1', 
        'arr': 'L'
    }
    
    try:
        if os.path.exists(path):
            config.read(path, encoding='utf-8')
            if 'CONTROL' in config:
                result = {}
                result['a_wh'] = config.get('CONTROL', 'a_warehouse', fallback=default_conf['a_wh']).strip()
                result['b_wh'] = config.get('CONTROL', 'b_warehouse', fallback=default_conf['b_wh']).strip()
                result['arr'] = config.get('CONTROL', 'arrow', fallback=default_conf['arr']).strip()
                return result
    except Exception:
        pass
    
    return default_conf

def calc_dist(start_pos, current_pos):
    if start_pos is None or current_pos is None:
        return 0.0
    
    dx = current_pos[0] - start_pos[0]
    dy = current_pos[1] - start_pos[1]
    return math.sqrt(dx*dx + dy*dy)

def calc_target_speed(remaining_distance, current_velocity):
    if remaining_distance <= 0.03:
        return 0.0
    
    speed = 0.1 + (remaining_distance - 0.1) * (0.3 - 0.1) / (0.4 - 0.1)
    return max(0.1, min(0.3, speed))