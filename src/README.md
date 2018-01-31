Node [/sawIntuitiveResearchKit_dvrk__home_cos_pmd_dvrk_nodocker_src_dvrk_ros_pmd_config_console_MTML_pmd_json]
Publications:
 * /dvrk/MTML_SIM/current_state [std_msgs/String]
 * /dvrk/MTML_SIM/desired_state [std_msgs/String]
 * /dvrk/MTML_SIM/error [std_msgs/String]
 * /dvrk/MTML_SIM/goal_reached [std_msgs/Bool]
 * /dvrk/MTML_SIM/gripper_closed_event [std_msgs/Bool]
 * /dvrk/MTML_SIM/gripper_pinch_event [std_msgs/Empty]
 * /dvrk/MTML_SIM/jacobian_body [std_msgs/Float64MultiArray]
 * /dvrk/MTML_SIM/jacobian_spatial [std_msgs/Float64MultiArray]
 * /dvrk/MTML_SIM/position_cartesian_current [geometry_msgs/PoseStamped]
 * /dvrk/MTML_SIM/position_cartesian_desired [geometry_msgs/PoseStamped]
 * /dvrk/MTML_SIM/position_cartesian_local_current [geometry_msgs/PoseStamped]
 * /dvrk/MTML_SIM/position_cartesian_local_desired [geometry_msgs/PoseStamped]
 * /dvrk/MTML_SIM/state_gripper_current [sensor_msgs/JointState]
 * /dvrk/MTML_SIM/state_joint_current [sensor_msgs/JointState]
 * /dvrk/MTML_SIM/state_joint_desired [sensor_msgs/JointState]
 * /dvrk/MTML_SIM/status [std_msgs/String]
 * /dvrk/MTML_SIM/twist_body_current [geometry_msgs/TwistStamped]
 * /dvrk/MTML_SIM/warning [std_msgs/String]
 * /dvrk/MTML_SIM/wrench_body_current [geometry_msgs/WrenchStamped]
 * /dvrk/console/teleop/scale [std_msgs/Float32]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /dvrk/MTML_SIM/lock_orientation [unknown type]
 * /dvrk/MTML_SIM/set_base_frame [unknown type]
 * /dvrk/MTML_SIM/set_cartesian_impedance_gains [unknown type]
 * /dvrk/MTML_SIM/set_desired_state [unknown type]
 * /dvrk/MTML_SIM/set_effort_joint [unknown type]
 * /dvrk/MTML_SIM/set_gravity_compensation [unknown type]
 * /dvrk/MTML_SIM/set_position_cartesian [unknown type]
 * /dvrk/MTML_SIM/set_position_goal_cartesian [unknown type]
 * /dvrk/MTML_SIM/set_position_goal_joint [unknown type]
 * /dvrk/MTML_SIM/set_position_joint [unknown type]
 * /dvrk/MTML_SIM/set_wrench_body [unknown type]
 * /dvrk/MTML_SIM/set_wrench_body_orientation_absolute [unknown type]
 * /dvrk/MTML_SIM/set_wrench_spatial [unknown type]
 * /dvrk/MTML_SIM/unlock_orientation [unknown type]
 * /dvrk/console/home [unknown type]
 * /dvrk/console/power_off [unknown type]
 * /dvrk/console/power_on [unknown type]
 * /dvrk/console/teleop/enable [unknown type]
 * /dvrk/console/teleop/set_scale [unknown type]

Services:
 * /sawIntuitiveResearchKit_dvrk__home_cos_pmd_dvrk_nodocker_src_dvrk_ros_pmd_config_console_MTML_pmd_json/get_loggers
 * /sawIntuitiveResearchKit_dvrk__home_cos_pmd_dvrk_nodocker_src_dvrk_ros_pmd_config_console_MTML_pmd_json/set_logger_level


contacting node http://stupid:34924/ ...
Pid: 15987
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /dvrk/MTML_SIM/state_joint_desired
    * to: /rqt_gui_py_node_32426
    * direction: outbound
    * transport: TCPROS
 * topic: /dvrk/MTML_SIM/state_gripper_current
    * to: /rqt_gui_py_node_32426
    * direction: outbound
    * transport: TCPROS
