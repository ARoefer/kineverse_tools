import numpy as np
import rospy

from argparse import ArgumentParser, ArgumentTypeError
from tqdm     import tqdm

from trajectory_msgs.msg import JointTrajectoryPoint as JointTrajectoryPointMsg

from kineverse.model.paths import Path
from kineverse.visualization.bpb_visualizer import ROSBPBVisualizer
from kineverse.utils     import res_pkg_path
from kineverse.urdf_fix  import urdf_filler, \
                                hacky_urdf_parser_fix
from kineverse.visualization.trajectory_visualizer import TrajectoryVisualizer
from urdf_parser_py.urdf import URDF

from kineverse_tools.ik_solver import IKSolver
from kineverse_tools.trajectory_processing import resample_spline
from kineverse_tools.srv import PlanIK as PlanIKSrv, PlanIKRequest, PlanIKResponse

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise ArgumentTypeError('Boolean value expected.')

if __name__ == '__main__':
    parser = ArgumentParser(description='Kineverse IK Solver')
    parser.add_argument('--urdf', '-r', help='Path to the robot urdf. '
                                             'Package paths are allowed')
    parser.add_argument('--link', help='Name of the link to solve for.')
    parser.add_argument('--vis', type=str2bool, default=False,
                                 help='Visualize the result to the '
                                      'kineverse_ik_solver/vis topic.')
    
    args = parser.parse_args()

    if args.link is None:
        print('No link was provided')
        exit()

    with open(res_pkg_path(args.urdf)) as f:
        urdf_str   = hacky_urdf_parser_fix(f.read())
        urdf_model = urdf_filler(URDF.from_xml_string(hacky_urdf_parser_fix(urdf_str)))

    rospy.init_node('kineverse_ik_solver')

    vis = None if not args.vis else ROSBPBVisualizer('~vis', base_frame='world')

    ik_solver = IKSolver(urdf_model, args.link, vis)

    def srv_plan_ik_cb(req: PlanIKRequest):
        joint_names = req.joint_state.name
        q_now  = req.joint_state.position
        x_goal = [req.goal.position.x,
                  req.goal.position.y,
                  req.goal.position.z,
                  req.goal.orientation.x,
                  req.goal.orientation.y,
                  req.goal.orientation.z,
                  req.goal.orientation.w]
        
        resp = PlanIKResponse()
        try:
            error, q_goal = ik_solver.solve(dict(zip(joint_names, q_now)), x_goal)
        except Exception as e:
            resp.failure = f'Solver crashed: {e}'
            return resp

        print('Final error: {}\nq_goal:\n  {}'.format(error, 
            '\n  '.join(sorted(f'{j}: {v}' for j, v in q_goal.items()))))

        cspline = np.array([[0] + q_now + [0] * len(q_now),
                            [2.0] + [q_goal[j] for j in joint_names] + [0] * len(q_now)])
    
        path = resample_spline(cspline, 100)[:, 1:len(joint_names) + 1].T

        trajectory = {ik_solver.km.get_data(f'robot/joints/{j}').position: positions for j, positions in zip(joint_names, path)}

        for x in tqdm(range(len(next(iter(trajectory.values()))))):
            state = {k: p[x] for k, p in trajectory.items()}

            ik_solver.collision_world.update_world(state)
            vis.begin_draw_cycle('trajectory')
            vis.draw_world('trajectory', ik_solver.collision_world, g=0.0, b=0.0)
            vis.render('trajectory')
            rospy.sleep(rospy.Duration(0.02))
        
        out_spline = resample_spline(cspline, 10)
        resp.error = error
        resp.trajectory.header.stamp = rospy.Time.now()
        resp.trajectory.joint_names  = joint_names
        
        for row in out_spline:
            point = JointTrajectoryPointMsg()
            point.time_from_start = row[0]
            point.positions  = list(row[1:len(joint_names) + 1])
            point.velocities = list(row[len(joint_names) + 1:])
            resp.trajectory.points.append(point)
        
        return resp

    srv_plan_ik = rospy.Service('~plan', PlanIKSrv, srv_plan_ik_cb)

    while not rospy.is_shutdown():
        rospy.sleep(1000)
    