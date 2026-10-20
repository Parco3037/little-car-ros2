"""Humble RPP profile; legacy DWB and base protection remain separate."""
import copy
import math


def controller_profile(cruise=0.25):
    if cruise not in (0.25, 0.35):
        raise ValueError('Only baseline 0.25 and trial 0.35 are supported')
    common = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': cruise, 'lookahead_dist': 0.12,
        'min_lookahead_dist': 0.08, 'max_lookahead_dist': 0.25,
        'lookahead_time': 1.0, 'use_velocity_scaled_lookahead_dist': True,
        'use_collision_detection': False,
        'use_cost_regulated_linear_velocity_scaling': False,
        'use_regulated_linear_velocity_scaling': True,
        'regulated_linear_scaling_min_radius': 0.5,
        'regulated_linear_scaling_min_speed': 0.04,
        'min_approach_linear_velocity': 0.02, 'approach_velocity_scaling_dist': 0.3,
        'rotate_to_heading_angular_vel': 0.25,
        'rotate_to_heading_min_angle': 0.15, 'max_angular_accel': 0.7,
        'transform_tolerance': 0.2, 'max_robot_pose_search_dist': 1.0,
        'use_interpolation': True,
    }
    forward = dict(common, use_rotate_to_heading=True, allow_reversing=False)
    reverse = dict(common, desired_linear_vel=0.08,
                   use_rotate_to_heading=False, allow_reversing=True)
    return {'controller_server': {'ros__parameters': {
        'use_sim_time': False, 'odom_topic': '/odom', 'controller_frequency': 10.0,
        'min_x_velocity_threshold': 0.001, 'min_y_velocity_threshold': 0.001,
        'min_theta_velocity_threshold': 0.001, 'failure_tolerance': 0.0,
        'progress_checker_plugin': 'progress_checker',
        'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                             'required_movement_radius': 0.02, 'movement_time_allowance': 30.0},
        'goal_checker_plugins': ['general_goal_checker'],
        'general_goal_checker': {'plugin': 'nav2_controller::SimpleGoalChecker',
                                'stateful': False, 'xy_goal_tolerance': 0.02,
                                'yaw_goal_tolerance': math.radians(10)},
        'controller_plugins': ['FixedForward', 'FixedReverse'],
        'FixedForward': forward, 'FixedReverse': reverse,
    }}}


def navigation_profile(legacy, cruise=0.25):
    controller_profile(cruise)
    result = copy.deepcopy(legacy)
    costmap = result['local_costmap']['local_costmap']['ros__parameters']
    # No obstacle/cost layers participate in fixed-route execution. Footprint
    # stays physical; its swept static check is mandatory when making a route.
    # Humble rcl YAML cannot type an empty parameter array. Keep one disabled
    # layer to obtain a valid free grid without any obstacle subscriptions.
    costmap['plugins'] = ['inflation_layer']
    costmap['inflation_layer']['enabled'] = False
    costmap['footprint_padding'] = 0.0
    costmap['track_unknown_space'] = False
    smooth = result['velocity_smoother']['ros__parameters']
    smooth['max_velocity'] = [cruise, 0.0, 0.35]
    smooth['min_velocity'] = [-0.08, 0.0, -0.35]
    return result
