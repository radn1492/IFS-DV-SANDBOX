"""My CONTROL node.   sb-launch control:=javier_pipeline/control

  in : /sandbox/path      nav_msgs/Path (map)
       /sandbox/pose      Odometry (map -> base_link), twist.linear.x = speed
  out: /control_command   fs_msgs/ControlCommand: throttle [0,1], brake [0,1], steering [-1,1] (+ = RIGHT)

Everything inside compute() is yours. Wheelbase 1.627 m, max steer 22.4 deg.
"""
import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body


def compute(path_body: np.ndarray, speed: float):
    """Path in the car frame + current speed -> (throttle, steering, brake).

    steering: + = left, like y in the car frame. tick() flips it for /control_command.
    
    for orientation: PID,PurePursuit,Stanley
    for velocity: PID
    """
    #PurePursuit
    L_min = 1
    k = 0.5
    L_d_max = 4

    L_d = np.clip(k * speed + L_min, L_min, L_d_max) #La distancia a la que ve
    d = np.hypot(path_body[:, 0], path_body[:, 1]) #Linea recta     
    ahead = (path_body[:, 0] > 0) & (d >= L_d) #Los puntos que están delante y por encima del l_d
    i = np.argmax(ahead) if ahead.any() else  len(d) - 1
    x,y = path_body[i]

    curvature = 2*y/(x**2 + y**2)
    steer = np.arctan(1.627 * curvature) / np.radians(22.4)
    K_v = 0.5

    if 0.2 <= abs(steer) <= 0.4 :
        brake = abs(0.1*steer)
        K_v = 0.4
    elif 0.4 <= abs(steer) <= 0.8 :
        brake = abs(0.2*steer)
        K_v = 0.3
    elif 0.8 <= abs(steer) <= 1 :
        brake = abs(0.3*steer)
        K_v = 0.2
    else:
        brake = 0.0
        K_v = 0.5
    
    throttle = float(np.clip(K_v * (3.5 - speed), 0, 1))   # target 3.5 m/s
    
    return throttle, float(np.clip(steer, -1, 1)), brake



class Control(Node):
    def __init__(self):
        super().__init__("control")
        self.path = self.pose = None
        self.create_subscription(Path, contract.PATH, lambda m: setattr(self, "path", m), 10)
        self.create_subscription(Odometry, contract.POSE, lambda m: setattr(self, "pose", m), 10)
        self.pub = self.create_publisher(ControlCommand, contract.CONTROL, 10)
        self.create_timer(1.0 / 40.0, self.tick)

    def tick(self):
        if self.path is None or self.pose is None or not self.path.poses:
            self.pub.publish(ControlCommand(brake=1.0))
            return
        pose = Pose2D.from_odometry(self.pose)
        pts = np.array([(p.pose.position.x, p.pose.position.y) for p in self.path.poses])
        throttle, steering, brake = compute(world_to_body(pts, pose), pose.vx)
        # /control_command uses the car's convention, + = RIGHT (like IFSSIM and
        # IFS09-DV-PIPELINE). compute() gives + = left (y is left): flip it.
        self.pub.publish(ControlCommand(throttle=float(throttle), steering=-float(steering), brake=float(brake)))


def main():
    run(Control)


if __name__ == "__main__":
    main()
