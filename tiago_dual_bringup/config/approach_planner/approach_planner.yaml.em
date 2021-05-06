disable_motion_planning: false
approach_planner:
  planning_groups: # Sorted by order of preference
@[if has_arm_left and has_arm_right]@
    - both_arms_torso
@[end if]@
@[if has_arm_left]@
    - arm_left_torso
    - arm_left
@[end if]@
@[if has_arm_right]@
    - arm_right_torso
    - arm_right
@[end if]@
    - torso
  exclude_from_planning_joints:
    - head_1_joint
    - head_2_joint
@[if end_effector_left == "pal-hey5"]@
    - hand_left_thumb_joint
    - hand_left_mrl_joint
    - hand_left_index_joint
@[end if]@
@[if end_effector_left == "pal-hey5"]@
    - hand_left_thumb_joint
    - hand_left_mrl_joint
    - hand_left_index_joint
@[end if]@
@[if end_effector_left == "pal-gripper"]@
    - gripper_left_left_finger_joint
    - gripper_left_right_finger_joint
@[end if]@
@[if end_effector_left in ["schunk-wsg", "robotiq-2f-85", "robotiq-2f-140"]]@
    - gripper_left_finger_joint
@[end if]@
@[if end_effector_right == "pal-hey5"]@
    - hand_right_thumb_joint
    - hand_right_mrl_joint
    - hand_right_index_joint
@[end if]@
@[if end_effector_right == "pal-hey5"]@
    - hand_right_thumb_joint
    - hand_right_mrl_joint
    - hand_right_index_joint
@[end if]@
@[if end_effector_right == "pal-gripper"]@
    - gripper_right_left_finger_joint
    - gripper_right_right_finger_joint
@[end if]@
@[if end_effector_right in ["schunk-wsg", "robotiq-2f-85", "robotiq-2f-140"]]@
    - gripper_right_finger_joint
@[end if]@
  joint_tolerance: 0.01
  skip_planning_approach_vel: 0.5
  skip_planning_approach_min_dur: 0.5
