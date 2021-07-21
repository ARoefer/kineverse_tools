#!/usr/bin/env python3
import rospy
import numpy as np
import tf
import sys

from argparse import ArgumentParser

import interactive_markers.interactive_marker_server as ims
from urdf_parser_py.urdf import Joint
from visualization_msgs.msg import Marker as MarkerMsg
from visualization_msgs.msg import InteractiveMarkerFeedback as InteractiveMarkerFeedbackMsg

from sensor_msgs.msg import JointState as JointStateMsg

from kineverse_tools.srv import PlanIK as PlanIKSrv, PlanIKRequest, PlanIKResponse
from kineverse_tools.utils import make6DOFGimbal

from kineverse_tools.srv import MoveToQPose as MoveToQPoseSrv,     \
                                               MoveToQPoseRequest, \
                                               MoveToQPoseResponse


from iiwa_msgs.msg import JointPosition as JointPositionMsg

if __name__ == '__main__':
    parser = ArgumentParser(description='IK toy can be used to interactively move a '
                                        'robot using RVIZ interactive markers.')
    parser.add_argument('--link', help='Name of the frame you want to control.')
    parser.add_argument('--base', help='Base frame of your robot.')
    args = parser.parse_args([a for a in sys.argv[1:] if ':=' not in a])

    if args.base is None:
        print('A base frame needs to be specified.')

    if args.link is None:
        print('A link frame needs to be specified.')

    rospy.init_node('ik_toy')

    srv_plan_ik    = rospy.ServiceProxy('kineverse_ik_solver/plan', PlanIKSrv)
    pub_joint_goal = rospy.Publisher('/iiwa/command/JointPosition', JointPositionMsg, queue_size=1, tcp_nodelay=True)
    # srv_move_to_q = rospy.ServiceProxy('iiwa_bridge/move_to_q', MoveToQPoseSrv)

    tf_listener = tf.TransformListener()

    try:
        print('Waiting for tool transform...')
        tf_listener.waitForTransform(f'/{args.base}', f'/{args.link}', rospy.Time(0), rospy.Duration(1.0))
        trans, quat = tf_listener.lookupTransform(f'/{args.base}', f'/{args.link}', rospy.Time(0))
        print('Got initial tool frame')
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(f'Could not fetch initial transform: {e}')
        exit()

    pose_changed = False
    joint_state  = None

    def process_js_update(js):
        global joint_state
        joint_state = js

    def process_marker_feedback(feedback: InteractiveMarkerFeedbackMsg):
        global pose_changed, planning_result
        if feedback.event_type == InteractiveMarkerFeedbackMsg.POSE_UPDATE:
            pose_changed = True

        if feedback.event_type == InteractiveMarkerFeedbackMsg.MOUSE_UP:
            if pose_changed:
                plan_req = PlanIKRequest()
                plan_req.joint_state = joint_state
                plan_req.goal = feedback.pose
                res = srv_plan_ik(plan_req)
                if res.failure == '':
                    q_goal = JointPositionMsg()
                    q_goal.header.stamp = rospy.Time.now()
                    q_goal.position.a1 = res.trajectory.points[-1].positions[0]
                    q_goal.position.a2 = res.trajectory.points[-1].positions[1]
                    q_goal.position.a3 = res.trajectory.points[-1].positions[2]
                    q_goal.position.a4 = res.trajectory.points[-1].positions[3]
                    q_goal.position.a5 = res.trajectory.points[-1].positions[4]
                    q_goal.position.a6 = res.trajectory.points[-1].positions[5]
                    q_goal.position.a7 = res.trajectory.points[-1].positions[6]
                    pub_joint_goal.publish(q_goal)


                    np_goal = np.array(res.trajectory.points[-1].positions)

                    print('Executing motion...')

                    while True:
                        np_state = np.array(joint_state.position)
                        if max(np.abs(np_goal - np_state)) < 1e-3:
                            break
                    
                    print('Motion complete')

                    # mres = srv_move_to_q(q_goal)
                    # print(f'Done. Motion failure message says: "{mres.failure}"')


    marker_server = ims.InteractiveMarkerServer('ik_toy_marker_server')

    intMarker = ims.InteractiveMarker()
    intMarker.name = 'ik_toy_goal'
    intMarker.header.frame_id = args.base
    intMarker.header.stamp = rospy.Time(0)
    intMarker.scale = 1.0
    intMarker.pose.position.x = trans[0]
    intMarker.pose.position.y = trans[1]
    intMarker.pose.position.z = trans[2]
    intMarker.pose.orientation.x = quat[0]
    intMarker.pose.orientation.y = quat[1]
    intMarker.pose.orientation.z = quat[2]
    intMarker.pose.orientation.w = quat[3]
    
    make6DOFGimbal(intMarker)

    marker_server.insert(intMarker, process_marker_feedback)
    marker_server.applyChanges()

    sub_js = rospy.Subscriber('/joint_states', JointStateMsg, process_js_update, queue_size=1)

    print('IK toy is ready')

    while not rospy.is_shutdown():
        rospy.sleep(1000)
