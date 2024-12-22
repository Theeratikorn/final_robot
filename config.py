import json
import numpy as np

# Load robot parameters from a JSON file
class RobotParameters:
    def __init__(self, config_file="robot_config.json"):
        # โหลดไฟล์ JSON
        with open(config_file, 'r') as file:
            config = json.load(file)

        # โหลดความยาวของลิงก์
        self.l1 = config['link_lengths']['l1']
        self.l2 = config['link_lengths']['l2']
        self.l3 = config['link_lengths']['l3']
        self.l4 = config['link_lengths']['l4']

        # โหลด phi และแปลงจากองศาเป็นเรเดียน
        self.phi = np.deg2rad(config['phi'])

        # โหลดขีดจำกัดของมุมและแปลงจากองศาเป็นเรเดียน
        self.jointLimitL_th1 = np.deg2rad(config['joint_limits']['th1']['left'])
        self.jointLimitR_th1 = np.deg2rad(config['joint_limits']['th1']['right'])

        self.jointLimitL_th2 = np.deg2rad(config['joint_limits']['th2']['left'])
        self.jointLimitR_th2 = np.deg2rad(config['joint_limits']['th2']['right'])

        self.jointLimitL_th3 = np.deg2rad(config['joint_limits']['th3']['left'])
        self.jointLimitR_th3 = np.deg2rad(config['joint_limits']['th3']['right'])

        self.jointLimitL_th4 = np.deg2rad(config['joint_limits']['th4']['left'])
        self.jointLimitR_th4 = np.deg2rad(config['joint_limits']['th4']['right'])

# สร้างอินสแตนซ์ของ RobotParameters
robot_params = RobotParameters()
