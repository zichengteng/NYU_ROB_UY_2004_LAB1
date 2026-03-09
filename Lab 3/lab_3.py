import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import scipy
np.set_printoptions(precision=3, suppress=True)

def rotation_x(angle):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(angle), -np.sin(angle), 0],
        [0, np.sin(angle), np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotation_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle), 0],
        [0, 1, 0, 0],
        [-np.sin(angle), 0, np.cos(angle), 0],
        [0, 0, 0, 1]
    ])

def rotation_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def translation(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        self.counter = 0
        # Options: 'forward', 'forward_fast', 'rotate', 'square'
        self.gait_mode = 'square'
        self.counter_step = 2 if self.gait_mode == 'forward_fast' else 1
        self.gait_counter_steps = {
            'forward': 1,
            'forward_fast': 2,
            'rotate': 2,
        }
        self.debug_log_stride = 25
        self.debug_counter = 0

        # Trotting gate positions
        ################################################################################################
        # TODO: Implement the trotting gait
        ################################################################################################
        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.bl_leg_fk]
        self.gait_caches = {}
        self.gait_counters = {}

        if self.gait_mode == 'square':
            # Repeat: move forward (one edge) then rotate in-place (one corner).
            self.square_sequence = [('forward_fast', 8), ('rotate', 3)] * 4
            self.square_segment_index = 0
            self.square_segment_step = 0

            for mode in ['forward_fast', 'rotate']:
                joint_cache, ee_cache = self.precompute_gait_cache(mode)
                self.gait_caches[mode] = (joint_cache, ee_cache)
                self.gait_counters[mode] = 0

            # Keep these members valid for existing debug prints.
            self.ee_triangle_positions = self.build_gait_waypoint_sequences('forward_fast')
            self.target_joint_positions_cache, self.target_ee_cache = self.gait_caches['forward_fast']
        else:
            self.ee_triangle_positions = self.build_gait_waypoint_sequences(self.gait_mode)
            self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()

        print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')


        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 100   # 100 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

    def build_triangle_points(self, base_width=0.10, height=0.09, base_center=np.array([0.0, 0.0, -0.14])):
        x_half = base_width / 2.0
        z_base = base_center[2]
        y_base = base_center[1]
        apex_z = z_base + height
        return {
            'touch_down': np.array([x_half, y_base, z_base]),
            'stand_1': np.array([0.5 * x_half, y_base, z_base]),
            'stand_2': np.array([0.0, y_base, z_base]),
            'stand_3': np.array([-0.5 * x_half, y_base, z_base]),
            'liftoff': np.array([-x_half, y_base, z_base]),
            'mid_swing': np.array([0.0, y_base, apex_z]),
        }

    def build_gait_waypoint_sequences(self, gait_mode):
        if gait_mode == 'forward_fast':
            points = self.build_triangle_points(base_width=0.14, height=0.09)
        elif gait_mode == 'rotate':
            points = self.build_triangle_points(base_width=0.12, height=0.08)
        else:
            points = self.build_triangle_points(base_width=0.10, height=0.09)

        td = points['touch_down']
        s1 = points['stand_1']
        s2 = points['stand_2']
        s3 = points['stand_3']
        lo = points['liftoff']
        ms = points['mid_swing']

        forward_a = np.array([td, s1, s2, s3, lo, ms])
        forward_b = np.array([s3, lo, ms, td, s1, s2])
        reverse_a = np.array([lo, s3, s2, s1, td, ms])
        reverse_b = np.array([s1, td, ms, lo, s3, s2])

        rf_ee_offset = np.array([0.06, -0.09, 0.0])
        lf_ee_offset = np.array([0.06, 0.09, 0.0])
        rb_ee_offset = np.array([-0.11, -0.09, 0.0])
        lb_ee_offset = np.array([-0.11, 0.09, 0.0])

        if gait_mode in ['forward', 'forward_fast']:
            rf = forward_a + rf_ee_offset
            lf = forward_b + lf_ee_offset
            rb = forward_b + rb_ee_offset
            lb = forward_a + lb_ee_offset
        elif gait_mode == 'rotate':
            rf = forward_a + rf_ee_offset
            rb = forward_b + rb_ee_offset
            lf = reverse_b + lf_ee_offset
            lb = reverse_a + lb_ee_offset
        else:
            raise ValueError(f'Unknown gait_mode: {gait_mode}')

        return [rf, lf, rb, lb]

    def precompute_gait_cache(self, gait_mode):
        self.ee_triangle_positions = self.build_gait_waypoint_sequences(gait_mode)
        return self.cache_target_joint_positions()

    def fr_leg_fk(self, theta):
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        T_LF_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_LF_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LF_0_ee = T_LF_0_1 @ T_LF_1_2 @ T_LF_2_3 @ T_LF_3_ee
        return T_LF_0_ee[:3, 3]

    def br_leg_fk(self, theta):
        T_RB_0_1 = translation(-0.07500, -0.07250, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RB_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RB_0_ee = T_RB_0_1 @ T_RB_1_2 @ T_RB_2_3 @ T_RB_3_ee
        return T_RB_0_ee[:3, 3]

    def bl_leg_fk(self, theta):
        T_LB_0_1 = translation(-0.07500, 0.07250, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_LB_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_LB_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_LB_3_ee = translation(0.06231, -0.06216, -0.01800)
        T_LB_0_ee = T_LB_0_1 @ T_LB_1_2 @ T_LB_2_3 @ T_LB_3_ee
        return T_LB_0_ee[:3, 3]

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])
        

    def listener_callback(self, msg):
        joints_of_interest = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3', 
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3', 
            'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3', 
            'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
        ]
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def get_error_leg(self, theta, desired_position):
        ################################################################################################
        # TODO: [already done] paste lab 3 inverse kinematics here
        ################################################################################################
        current_position = self.leg_forward_kinematics(theta)
        error = np.asarray(current_position) - np.asarray(desired_position)
        return float(np.dot(error, error))

    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
        self.leg_forward_kinematics = self.fk_functions[leg_index]
        ################################################################################################
        # TODO: implement interpolation for all 4 legs here
        ################################################################################################
        result = scipy.optimize.minimize(
            self.get_error_leg,
            np.asarray(initial_guess, dtype=float),
            args=(np.asarray(target_ee, dtype=float),),
            method='BFGS'
        )
        return result.x

    def interpolate_triangle(self, t, leg_index):
        ################################################################################################
        # TODO: implement interpolation for all 4 legs here
        ################################################################################################
        triangle_positions = self.ee_triangle_positions[leg_index]
        num_waypoints = triangle_positions.shape[0]

        t = float(t) % 1.0
        phase = t * num_waypoints
        segment_index = int(np.floor(phase))
        segment_progress = phase - segment_index

        start_point = triangle_positions[segment_index % num_waypoints]
        end_point = triangle_positions[(segment_index + 1) % num_waypoints]
        return (1.0 - segment_progress) * start_point + segment_progress * end_point

    def cache_target_joint_positions(self):
        # Calculate and store the target joint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(4):
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            for t in np.arange(0, 1, 0.02):# 0.02):
                #print(t)
                target_ee = self.interpolate_triangle(t, leg_index)
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)

                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12)
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)
        
        return target_joint_positions_cache, target_ee_cache

    def get_target_joint_positions(self):
        if self.gait_mode == 'square':
            return self.get_square_target_joint_positions()

        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += self.counter_step
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = self.counter % self.target_joint_positions_cache.shape[0]
        return target_ee, target_joint_positions

    def get_square_target_joint_positions(self):
        mode, segment_cycles = self.square_sequence[self.square_segment_index]
        joint_cache, ee_cache = self.gait_caches[mode]
        mode_step = self.gait_counter_steps[mode]

        idx = self.gait_counters[mode]
        target_joint_positions = joint_cache[idx]
        target_ee = ee_cache[idx]
        self.gait_counters[mode] = (idx + mode_step) % joint_cache.shape[0]

        self.square_segment_step += 1
        steps_per_cycle = int(np.ceil(joint_cache.shape[0] / mode_step))
        segment_total_steps = segment_cycles * steps_per_cycle
        if self.square_segment_step >= segment_total_steps:
            prev_mode = mode
            self.square_segment_step = 0
            self.square_segment_index = (self.square_segment_index + 1) % len(self.square_sequence)
            next_mode = self.square_sequence[self.square_segment_index][0]
            if next_mode != prev_mode:
                self.get_logger().info(f'Square gait switch: {prev_mode} -> {next_mode}')

        return target_ee, target_joint_positions

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee, self.target_joint_positions = self.get_target_joint_positions()
            current_ee = self.forward_kinematics(self.joint_positions)
            self.debug_counter += 1
            if self.debug_counter % self.debug_log_stride == 0:
                self.get_logger().info(
                    f'Mode: {self.gait_mode}, Target EE: {target_ee}, \
                    Current EE: {current_ee}, \
                    Target Angles: {self.target_joint_positions}, \
                    Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
                    Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
