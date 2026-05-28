"""Publica una caja como CollisionObject en la PlanningScene de MoveIt.
Una vez ejecutado, el obstaculo aparece en RViz (panel Scene Objects)
y el planificador lo evita automaticamente."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("irb120_obstacle_publisher")

        # Transient local para que MoveIt reciba el ultimo mensaje aunque
        # se suscriba despues.
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(PlanningScene, "/planning_scene", qos)

        # Republica cada segundo durante un rato por si la escena se
        # reinicia (p.ej. al volver a abrir RViz).
        self.timer = self.create_timer(1.0, self._publish_once)
        self._published_count = 0

    def _build_scene(self) -> PlanningScene:
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.15, 0.15, 0.4]   # x, y, z (m)

        pose = Pose()
        pose.position.x = 0.45
        pose.position.y = 0.0
        pose.position.z = 0.20
        pose.orientation.w = 1.0

        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "obstacle_box"
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        world = PlanningSceneWorld()
        world.collision_objects.append(obj)

        scene = PlanningScene()
        scene.world = world
        scene.is_diff = True
        return scene

    def _publish_once(self):
        self.pub.publish(self._build_scene())
        self._published_count += 1
        if self._published_count == 1:
            self.get_logger().info(
                "Obstaculo 'obstacle_box' publicado en (0.45, 0.0, 0.20)"
            )
        if self._published_count >= 10:
            self.timer.cancel()
            self.get_logger().info("Republicacion finalizada. Nodo seguira vivo.")


def main():
    rclpy.init()
    node = ObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
