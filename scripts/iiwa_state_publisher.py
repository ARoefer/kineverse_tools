#!/usr/bin/env python3
import rospy
from multiprocessing import RLock

from sensor_msgs.msg import JointState as JointStateMsg

from iiwa_msgs.msg import JointPosition as JointPositionMsg
from iiwa_msgs.msg import JointVelocity as JointVelocityMsg
from iiwa_msgs.msg import JointTorque   as JointTorqueMsg


class IIWAStateRepublisher(object):
    def __init__(self):
        self.last_position = None
        self.last_velocity = None
        self.last_torque   = None

        self.lock = RLock()
        self.current_js = JointStateMsg()
        self.current_js.name = [f'joint_a{x}' for x in range(1, 8)] 

        self.pub_js = rospy.Publisher('/joint_states', JointStateMsg, queue_size=1, tcp_nodelay=True)

        self.sub_pos = rospy.Subscriber('/iiwa/state/JointPosition', JointPositionMsg, callback=self.cb_position, queue_size=1)
        self.sub_vel = rospy.Subscriber('/iiwa/state/JointVelocity', JointVelocityMsg, callback=self.cb_velocity, queue_size=1)
        self.sub_torque = rospy.Subscriber('/iiwa/state/JointTorque', JointTorqueMsg, callback=self.cb_torque, queue_size=1)

        rate = rospy.get_param('~rate', 100)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.cb_publish)


    def cb_publish(self, *args):
        if self.last_position is not None and self.last_velocity is not None and self.last_torque is not None:
            with self.lock:
                self.current_js.position = [self.last_position.position.a1,
                                            self.last_position.position.a2,
                                            self.last_position.position.a3,
                                            self.last_position.position.a4,
                                            self.last_position.position.a5,
                                            self.last_position.position.a6,
                                            self.last_position.position.a7]
                self.current_js.velocity = [self.last_velocity.velocity.a1,
                                            self.last_velocity.velocity.a2,
                                            self.last_velocity.velocity.a3,
                                            self.last_velocity.velocity.a4,
                                            self.last_velocity.velocity.a5,
                                            self.last_velocity.velocity.a6,
                                            self.last_velocity.velocity.a7]
                self.current_js.effort   = [self.last_torque.torque.a1,
                                            self.last_torque.torque.a2,
                                            self.last_torque.torque.a3,
                                            self.last_torque.torque.a4,
                                            self.last_torque.torque.a5,
                                            self.last_torque.torque.a6,
                                            self.last_torque.torque.a7]
            
            self.current_js.header.stamp = rospy.Time.now()
            self.pub_js.publish(self.current_js)

    def cb_position(self, pos: JointPositionMsg):
        with self.lock:
            self.last_position = pos

    def cb_velocity(self, vel: JointVelocityMsg):
        with self.lock:
            self.last_velocity = vel

    def cb_torque(self, torque: JointTorqueMsg):
        with self.lock:
            self.last_torque = torque



if __name__ == '__main__':
    rospy.init_node('iiwa_state_publisher')

    publisher = IIWAStateRepublisher()

    while not rospy.is_shutdown():
        rospy.sleep(10)
