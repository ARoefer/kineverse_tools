import rospy
import kineverse.gradients.gradient_math as gm
import numpy as np

from collections import namedtuple
from kineverse.model.frames     import Frame
from kineverse.model.paths      import Path, find_all_of_type
from kineverse.ros.tf_publisher import ModelTFBroadcaster_URDF
from kineverse.utils            import generate_transition_function
from kineverse_msgs.msg         import ValueMap as ValueMapMsg

from sensor_msgs.msg import JointState as JointStateMsg

class StateContainer(object):
    def __init__(self):
        self.position = None
        self.velocity = None

DT_SYM = gm.Symbol('dt')

class KineverseKinematicSim(object):
    def __init__(self, km, 
                       model_path, 
                       urdf_param='~robot_description',
                       state_topic='~joint_states',
                       command_topic='~command',
                       integration_rules=None,
                       update_rate=50,
                       command_type=JointStateMsg,
                       watchdog_period=0.2):
        self.model = km.get_data(model_path)
        self.broadcaster = ModelTFBroadcaster_URDF(urdf_param, model_path, self.model)

        state_symbols   = set()
        control_symbols = set()
        for f_path in find_all_of_type(model_path, self.model, Frame):
            frame = km.get_data(f_path)
            state_symbols.update(gm.free_symbols(frame.pose))
            control_symbols.update(gm.get_diff_symbols(frame.pose))

        self.state_vars, \
        self.transition_function, \
        self.controls_vector = generate_transition_function(DT_SYM,
                                                            state_symbols, 
                                                            integration_rules)

        self.state_map  = {s: x for x, s in enumerate(self.controls_vector)}
        self.flat_state = np.zeros(len(self.state_map))
        self.flat_state[0] = 1.0 / update_rate

        self.vel_watchdogs = [None] * len(self.state_map)
        self.watchdog_period = rospy.Duration(watchdog_period)

        self.state_info = {}

        for s in self.state_map.keys():
            if gm.get_symbol_type(s) in {gm.TYPE_POSITION, gm.TYPE_VELOCITY}:
                s_str_typeless = str(Path(gm.erase_type(s))[-1])
                if s_str_typeless not in self.state_info:
                    self.state_info[s_str_typeless] = StateContainer()

                if gm.get_symbol_type(s) == gm.TYPE_POSITION:
                    self.state_info[s_str_typeless].position = self.state_map[s]
                elif gm.get_symbol_type(s) == gm.TYPE_VELOCITY:
                    self.state_info[s_str_typeless].velocity = self.state_map[s]

        print('Command variables:\n  {}'.format('\n  '.join(sorted(self.state_info.keys()))))

        self.template_msg = JointStateMsg()
        self.template_msg.name = list(self.state_info.keys())
        self.template_msg.position = [0] * len(list(self.state_info.keys()))
        self.template_msg.velocity = [0] * len(list(self.state_info.keys()))

        self.pub_state = rospy.Publisher(state_topic, JointStateMsg, queue_size=1, tcp_nodelay=True)
        self.sub_cmd   = rospy.Subscriber(command_topic, command_type, callback=self.process_command, queue_size=1)
        self.timer_update = rospy.Timer(rospy.Duration(self.flat_state[0]), self.cb_update)

    def process_command(self, msg):
        now = rospy.Time.now()

        if type(msg) == JointStateMsg:
            for x, name in enumerate(msg.name):
                if name != str(DT_SYM) and name in self.state_info:
                    info = self.state_info[name]

                    if len(msg.position) > x:
                        if info.position is not None:
                            self.flat_state[info.position] = msg.position[x]
                            # print(f'Set position for {name}: {msg.position[x]}')
                    if len(msg.velocity) > x:
                        if info.velocity is not None:
                            self.flat_state[info.velocity] = msg.velocity[x]
                            self.vel_watchdogs[info.velocity] = now
                            # print(f'Set velocity for {name} ({info.velocity}): {msg.velocity[x]}')
        elif type(msg) == ValueMapMsg:
            for str_symbol, value in zip(msg.symbol, msg.value):
                sym = gm.Symbol(str_symbol)
                if str_symbol != str(DT_SYM) and sym in self.state_map:
                    x = self.state_map[sym]
                    self.flat_state[x] = value
                    if gm.get_symbol_type(sym) == gm.TYPE_VELOCITY:
                        self.vel_watchdogs[x] = now
                else:
                    print(f'Command attempted to set {sym} but that variable is unknown. '
                           'Vars are:\n  {}'.format('\n  '.join(str(s) for s in self.state_map.keys())))

    def cb_update(self, *args):
        now = rospy.Time.now()
        for x, stamp in enumerate(self.vel_watchdogs):
            if stamp is not None and (now - stamp) > self.watchdog_period:
                self.flat_state[x] = 0
                self.vel_watchdogs[x] = None

        update = self.transition_function.call2(self.flat_state).flatten()
        for s, v in zip(self.state_vars, update):
            self.flat_state[self.state_map[s]] = v

        for x, name in enumerate(self.template_msg.name):
            info = self.state_info[name]
            if info.position is not None:
                self.template_msg.position[x] = self.flat_state[info.position]
            else:
                self.template_msg.position[x] = 0

            if info.velocity is not None:
                self.template_msg.velocity[x] = self.flat_state[info.velocity]
            else:
                self.template_msg.velocity[x] = 0

        self.template_msg.header.stamp = now
        self.pub_state.publish(self.template_msg)

        self.broadcaster.update_state(dict(zip(self.controls_vector, self.flat_state)))
        self.broadcaster.publish_state()