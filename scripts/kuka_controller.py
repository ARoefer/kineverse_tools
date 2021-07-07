#!/usr/bin/env python3
import rospy
import numpy as np
from multiprocessing import RLock

from robot_io.kuka_iiwa.iiwa_controller import IIWAController

from sensor_msgs.msg import JointState as JointStateMsg

from kineverse_tools.srv import MoveToQPose as MoveToQPoseSrv,     \
                                               MoveToQPoseRequest, \
                                               MoveToQPoseResponse

class IIWAROSBridge(object):
    def __init__(self):
        self._info_lock = RLock()

        self.iiwa = IIWAController(use_impedance=True)
        self.iiwa._send_init_message()

        self._js_timer = rospy.Timer(rospy.Duration(0.04), self.cb_js_timer)
        self._latest_info = None
        self._js_prototype = JointStateMsg()
        self._js_prototype.name = [f'joint_a{x}' for x in range(1, 8)]

        self.pub_js = rospy.Publisher('~joint_states', JointStateMsg, queue_size=1, tcp_nodelay=True)
        self.srv_move_to_q = rospy.Service('~move_to_q', MoveToQPoseSrv, self.cb_srv_move_to_q)

    def cb_js_timer(self, *args):
        with self._info_lock:
            self._latest_info = self.iiwa.getInfo()

        self._js_prototype.position = list(self._latest_info['joint_positions'])

        self.pub_js.publish(self._js_prototype)

    def cb_srv_move_to_q(self, req: MoveToQPoseRequest):
        self.iiwa.send_joint_angles_rad(tuple(req.pose.position))
        np_goal = np.array(req.pose.position)
        while True:
            with self._info_lock:
                if np.max(np.abs(np_goal - self._latest_info)) < 1e-3:
                    break
            rospy.sleep(0.02)
        
        return MoveToQPoseResponse()


if __name__ == "__main__":
    rospy.init_node('iiwa_robot')

    bridge = IIWAROSBridge()

    print('IIWA bridge is ready')

    while not rospy.is_shutdown():
        rospy.sleep(1000)

