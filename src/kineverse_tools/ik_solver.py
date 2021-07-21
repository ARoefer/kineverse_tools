import traceback
from typing import Iterable

import kineverse.gradients.gradient_math as gm

from kineverse.model.paths                import Path
from kineverse.model.geometry_model       import GeometryModel
from kineverse.motion.min_qp_builder      import GeomQPBuilder as GQPB, \
                                                 SoftConstraint,        \
                                                 generate_controlled_values
from kineverse.motion.integrator          import CommandIntegrator
from kineverse.operations.urdf_operations import load_urdf


def ik_solve_one_shot(km, actuated_pose, q_now, goal_pose, visualizer=None, step_limit=50):
    err_rot = gm.norm(gm.rot_of(goal_pose - actuated_pose).elements())
    err_lin = gm.norm(gm.pos_of(goal_pose - actuated_pose))

    active_symbols = {s for s in gm.free_symbols(actuated_pose) if gm.get_symbol_type(s) == gm.TYPE_POSITION}
    controlled_symbols = {gm.DiffSymbol(s) for s in active_symbols}

    controlled_values, constraints = generate_controlled_values(km.get_constraints_by_symbols(controlled_symbols.union(active_symbols)),
                                                                controlled_symbols)

    collision_world = km.get_active_geometry(active_symbols)

    goal_lin = SoftConstraint(-err_lin, -err_lin, 1.0, err_lin)
    goal_ang = SoftConstraint(-err_rot, -err_rot, 0.1, err_rot)

    qp = GQPB(collision_world, 
              constraints,
              {'IK_constraint_lin': goal_lin,
               'IK_constraint_ang': goal_ang},
              controlled_values,
              visualizer=visualizer)


    integrator = CommandIntegrator(qp, start_state=q_now)

    try:
        integrator.restart(title='IK integrator')
        integrator.run(dt=0.5, max_iterations=step_limit, logging=False)
        return integrator.get_latest_error(), integrator.state
    except Exception as e:
        traceback.print_exception(type(e), e, e.__traceback__)
        print(f'IK-Solver crashed:\n{e}')

    return 1.0, {}



class IKSolver(object):
    def __init__(self, urdf_model, link_name, visualizer=None):
        self.km = GeometryModel()
        load_urdf(self.km, Path('robot'), urdf_model)

        self.km.clean_structure()
        self.km.dispatch_events()

        self.goal_x, self.goal_y, self.goal_z = [gm.Symbol(f'goal_{s}') for s in 'xyz']
        self.goal_qx, self.goal_qy, self.goal_qz, self.goal_qw = [gm.Symbol(f'goal_q{s}') for s in 'xyzw']

        self.goal_vars = [self.goal_x,
                          self.goal_y,
                          self.goal_z,
                          self.goal_qx,
                          self.goal_qy,
                          self.goal_qz,
                          self.goal_qw]

        self.goal_pose = gm.frame3_quaternion(*self.goal_vars)

        eef = self.km.get_data(f'robot/links/{link_name}/pose')

        # Simple 9D error. Rotation will dominate, but let's see
        err_rot = gm.norm(gm.rot_of(self.goal_pose - eef).elements())
        err_lin = gm.norm(gm.pos_of(self.goal_pose - eef))

        self.joint_symbols = [j.position for j in self.km.get_data('robot/joints').values() if hasattr(j, 'position') and gm.is_symbol(j.position)]
        controlled_symbols = {gm.DiffSymbol(s) for s in self.joint_symbols}

        controlled_values, constraints = generate_controlled_values(self.km.get_constraints_by_symbols(controlled_symbols.union(self.joint_symbols)),
                                                                    controlled_symbols)
        self.collision_world = self.km.get_active_geometry(self.joint_symbols)

        goal_lin = SoftConstraint(-err_lin, -err_lin, 1.0, err_lin)
        goal_ang = SoftConstraint(-err_rot, -err_rot, 0.1, err_rot)

        self.qp = GQPB(self.collision_world, 
                       constraints,
                       {'IK_constraint_lin': goal_lin,
                        'IK_constraint_ang': goal_ang},
                       controlled_values,
                       visualizer=visualizer)

    def solve(self, q_now : dict, p_goal : Iterable, step_limit=50):
        """Attempts to solve IK for the given pose, 
           starting at a given robot configuration.

        Args:
            q_now (dict): Dict mapping joint names to scalars
            p_goal (iterable): An iterable holding a quaternion as [x,y,z,qx,qy,qz,qw]
            step_limit (int): Maximum number of steps the solver will take

        Returns:
            (float, dict): A tuple containing the final error of the solution and
                           the final robot configuration.
        """
        try:
            rev_map     = {self.km.get_data(f'robot/joints/{j}').position: j for j in q_now}
            start_state = {j: q_now[k] for j, k in rev_map.items()}
            print(rev_map)
        except KeyError as e:
            raise Exception(f'Joint {k} does not seem to be a part of the robot. Original error:\n  {e}')
        
        start_state.update(dict(zip(self.goal_vars, p_goal)))

        self.integrator = CommandIntegrator(self.qp, start_state=start_state)
        
        try:
            self.integrator.restart(title='IK integrator')
            self.integrator.run(dt=0.5, max_iterations=step_limit, logging=False)
            return self.integrator.get_latest_error(), {rev_map[j]: v for j, v in self.integrator.state.items() 
                                                                          if j in rev_map}
        except Exception as e:
            traceback.print_exception(type(e), e, e.__traceback__)
            print(f'IK-Solver crashed:\n{e}')

        return 1.0, {}