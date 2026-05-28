"""Planifica con MoveIt a la posicion publicada por la herramienta
'Publish Point' de RViz (topic /clicked_point). La orientacion del
TCP se fija apuntando hacia abajo (Z- del mundo)."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from moveit.planning import MoveItPy
from irb120_jazzy_sim._moveit_config import build_moveit_config_dict


class ClickedPointPlanner(Node):
    def __init__(self, moveit: MoveItPy):
        super().__init__("irb120_clicked_planner")
        self.moveit = moveit
        self.arm = moveit.get_planning_component("irb120_arm")
        self.sub = self.create_subscription(
            PointStamped, "/clicked_point", self.on_point, 10
        )
        self.get_logger().info(
            "Listo. Usa 'Publish Point' en RViz para enviar un objetivo."
        )

    def on_point(self, msg: PointStamped):
        self.get_logger().info(
            f"Punto recibido: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f})"
        )
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id or "world"
        pose.pose.position = msg.point
        # TCP apuntando hacia abajo (orientacion fija razonable)
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")

        plan = self.arm.plan()
        if plan:
            self.moveit.execute(plan.trajectory, controllers=[])
            self.get_logger().info("Trayectoria ejecutada")
        else:
            self.get_logger().warn("No se pudo planificar a ese punto")


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="irb120_clicked_moveit_py",
                      config_dict=build_moveit_config_dict())
    node = ClickedPointPlanner(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
