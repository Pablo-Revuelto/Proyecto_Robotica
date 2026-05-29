import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

from irb120_jazzy_sim._moveit_config import build_moveit_config_dict


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="irb120_moveit_py",
                      config_dict=build_moveit_config_dict())
    arm = moveit.get_planning_component("irb120_arm")

    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    # Orden: joint_1 ... joint_6 (segun el grupo irb120_arm en el SRDF)
    joint_goal = [0.0, -0.7, 0.9, 0.0, 1.0, 0.0]
    robot_state.set_joint_group_positions("irb120_arm", joint_goal)
    # Imprescindible: recalcula las transformadas del estado. Sin esto,
    # set_goal_state(robot_state=...) accede a transforms sin actualizar y
    # provoca un segfault (SIGSEGV) en moveit_py.
    robot_state.update()

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=robot_state)

    plan_result = arm.plan()
    if plan_result:
        moveit.execute(plan_result.trajectory, controllers=[])
        print("Trayectoria ejecutada")
    else:
        print("No se pudo planificar")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
