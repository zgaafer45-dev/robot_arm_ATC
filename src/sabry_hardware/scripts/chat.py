#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Point
from sabry_hardware.srv import ChangeTool, LinearMotor
from sabry_hardware.action import ChangeTool
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import trimesh
from shape_msgs.msg import Mesh, MeshTriangle
import os
from ament_index_python.packages import get_package_share_directory
# from gazebo_msgs.srv import SpawnEntity, DeleteEntity

class ToolChangeManager(Node):

    def __init__(self):
        super().__init__("tool_change_manager")

        # Action server
        self._action_server = ActionServer(
            self,
            ChangeTool,
            "change_tool",
            self.execute_callback
        )

        # Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.scene_client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.tool_client = self.create_client(LinearMotor, "tool_changer/set_state")
        # self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        # self.delete_client = self.create_client(DeleteEntity, "/delete_entity")

        # Wait for dependencies
        self.move_client.wait_for_server()
        self.scene_client.wait_for_service()
        self.tool_client.wait_for_service()
        # self.spawn_client.wait_for_service()
        # self.delete_client.wait_for_service()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tool_poses = {
            'gripper': {'dock': 'gripper_docking_point', 'mount': 'gripper_mount'},
            'screwdriver': {'dock': 'screwdriver_docking_point', 'mount': 'screwdriver_mount'},
            'camera': {'dock': 'camera_docking_point', 'mount': 'camera_mount'}
        }

        # State
        self.state = "IDLE"
        self.current_tool = None
        self.goal_handle = None
        self._result = None
        self.goal_active = False

        self.get_logger().info("ToolChangeManager ready")

    def create_mesh(self, mesh_path):

        try:
            mesh = trimesh.load(mesh_path)

            mesh_msg = Mesh()

            # Add triangles
            for face in mesh.faces:
                triangle = MeshTriangle()
                triangle.vertex_indices = [int(face[0]), int(face[1]), int(face[2])]
                mesh_msg.triangles.append(triangle)

            # Add vertices
            for vertex in mesh.vertices:
                point = Point()
                point.x = float(vertex[0])
                point.y = float(vertex[1])
                point.z = float(vertex[2])
                mesh_msg.vertices.append(point)

            return mesh_msg

        except Exception as e:
            self.get_logger().error(f"Failed to load mesh: {e}")
            return None

    
    def execute_callback(self, goal_handle):

        if self.state != "IDLE":
            result = ChangeTool.Result()
            result.success = False
            result.message = "Busy"
            goal_handle.abort()
            return result
    
        self.goal_handle = goal_handle
        self.goal_active = True
        self._result = None
    
        tool_name = goal_handle.request.tool_name
    
        if tool_name == "gripper":
            self.start_attach_sequence()
        elif tool_name == "none":
            if self.current_tool is None:
                result = ChangeTool.Result()
                result.success = False
                result.message = "No tool attached"
                goal_handle.abort()
                return result
            self.start_detach_sequence()
        else:
            result = ChangeTool.Result()
            result.success = False
            result.message = "Unsupported tool"
            goal_handle.abort()
            return result
    
        # BLOCK HERE using rclpy spinning
        while self._result is None:
            rclpy.spin_once(self, timeout_sec=0.1)
    
        return self._result

    def publish_feedback(self, text):
        
        feedback = ChangeTool.Feedback()
        feedback.current_state = text
        self.goal_handle.publish_feedback(feedback)

    def delete_tool_from_dock(self, tool_name):

        req = DeleteEntity.Request()
        req.name = tool_name

        future = self.delete_client.call_async(req)
        future.add_done_callback(
            lambda f: self.spawn_tool_on_robot(tool_name)
        )
    
    def spawn_tool_on_robot(self, tool_name):

        req = SpawnEntity.Request()
        req.name = tool_name
        req.robot_namespace = ""

        pkg_path = get_package_share_directory("sabry")
        urdf_path = os.path.join(pkg_path, "urdf", "mock_tool.xacro")

        with open(urdf_path, "r") as f:
            req.xml = f.read()

        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.0

        req.reference_frame = "sabry::tool_mount_link"

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_done_cb)

    def spawn_done_cb(self, future):

        if future.result() is None:
            self.abort("Failed to spawn tool on robot")
            return

        self.get_logger().info("Tool spawned on robot")

        # Now update MoveIt
        self.attach_gripper()

    def delete_attached_tool(self, tool_name):

        req = DeleteEntity.Request()
        req.name = tool_name

        future = self.delete_client.call_async(req)
        future.add_done_callback(
            lambda f: self.spawn_tool_at_dock(tool_name)
        )

    def spawn_tool_at_dock(self, tool_name):

        req = SpawnEntity.Request()
        req.name = tool_name

        pkg_path = get_package_share_directory("sabry")
        urdf_path = os.path.join(pkg_path, "urdf", "mock_tool.xacro")

        with open(urdf_path, "r") as f:
            req.xml = f.read()

        req.initial_pose.position.x = 0.193
        req.initial_pose.position.y = -0.287
        req.initial_pose.position.z = 0.238

        req.reference_frame = "world"

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.detach_spawn_done_cb)

    def detach_spawn_done_cb(self, future):

        if future.result() is None:
            self.abort("Failed to respawn tool at dock")
            return

        self.get_logger().info("Tool returned to dock")

        self.detach_gripper()

    # ==========================================================
    # STATE MACHINE START
    # ==========================================================
    def start_attach_sequence(self):
        self.get_logger().info("Starting gripper pickup sequence")
        self.publish_feedback("Moving to approach position")

        self.state = "MOVE_APPROACH"
        # self.send_move(self.offset_pose(self.get_dock_pose(), dz=0.10))
        self.send_move(self.offset_pose(self.get_transform('base_link', self.tool_poses['gripper']['mount']), dz=0.10))

    def start_detach_sequence(self):
        self.get_logger().info("Starting gripper detach sequence")
        self.publish_feedback("Staring detach")

        self.state = "DETACH_MOVE_APPROACH"

        pose = self.get_transform('world', self.tool_poses['gripper']['mount'])
        if pose is None:
            self.abort("Dock transform not available")
            return

        self.send_move(self.offset_pose(pose, dz=0.10))

    # ==========================================================
    # MOVE HANDLING
    # ==========================================================
    def send_move(self, pose: PoseStamped):
        self.get_logger().info(f"Sending move to: "f"x={pose.pose.position.x:.4f}, "f"y={pose.pose.position.y:.4f}, " f"z={pose.pose.position.z:.4f}")
        goal = self.create_goal(pose)
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
    
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.abort("Move rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_cb)

    def move_result_cb(self, future):
        if not self.goal_active:
            return
        result = future.result().result
        if result.error_code.val != 1:
            self.abort(f"Move failed {result.error_code.val}")
            return

        # Advance state
        if self.state == "MOVE_APPROACH":
            self.state = "MOVE_DOCK"
            # self.send_move(self.get_dock_pose())
            self.publish_feedback("Docking tool")
            self.send_move(self.get_transform('base_link', self.tool_poses['gripper']['mount']))

        elif self.state == "MOVE_DOCK":
            self.publish_feedback("Unlocking tool")
            self.state = "UNLOCK"
            self.send_tool_command(2)

        elif self.state == "MOVE_LIFT":
            self.publish_feedback("Lifting tool")
            self.finish_success()
            
        elif self.state == "DETACH_MOVE_APPROACH":
            self.state = "DETACH_MOVE_DOCK"
            self.publish_feedback("Undocking tool")
            pose = self.get_transform('world', self.tool_poses['gripper']['mount'])
            if pose is None:
                self.abort("Dock transform not available")
                return
            self.send_move(pose)

        elif self.state == "DETACH_MOVE_DOCK":
            self.state = "DETACH_UNLOCK"
            self.publish_feedback("Unlocking tool")
            self.send_tool_command(2)

        elif self.state == "DETACH_MOVE_LIFT":
            self.finish_detach_success()

    # ==========================================================
    # TOOL ACTUATOR
    # ==========================================================
    def send_tool_command(self, cmd):
        req = LinearMotor.Request()
        req.command = cmd
        future = self.tool_client.call_async(req)
        future.add_done_callback(self.tool_result_cb)

    def tool_result_cb(self, future):
        if not self.goal_active:
            return
        result = future.result()
        if result is None or not result.success:
            self.abort("Tool command failed")
            return

        if self.state == "UNLOCK":
            self.state = "ATTACH"
            self.attach_gripper()
            # self.delete_tool_from_dock("gripper")

        elif self.state == "LOCK":
            self.state = "MOVE_LIFT"
            # self.send_move(self.offset_pose(self.get_dock_pose(), dz=0.15))
            self.send_move(self.offset_pose(self.get_transform('base_link', self.tool_poses['gripper']['mount']), dz=0.15))

        elif self.state == "DETACH_UNLOCK":
            self.state = "DETACH_REMOVE"
            self.detach_gripper()
            # self.delete_attached_tool("gripper")

        elif self.state == "DETACH_LOCK":
            self.state = "DETACH_MOVE_LIFT"
            pose = self.get_transform('world', self.tool_poses['gripper']['mount'])
            if pose is None:
                self.abort("Dock transform not available")
                return
            self.send_move(self.offset_pose(pose, dz=0.15))

    # ==========================================================
    # PLANNING SCENE
    # ==========================================================
    def attach_gripper(self):
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True

        co = CollisionObject()
        co.id = "gripper"
        co.header.frame_id = "tool_mount_link"

        # primitive = SolidPrimitive()
        # primitive.type = SolidPrimitive.CYLINDER
        # primitive.dimensions = [0.12, 0.012]

        # co.primitives.append(primitive)
        # co.primitive_poses.append(self.relative_pose().pose)

        pkg_path = get_package_share_directory("sabry")
        mesh_path = os.path.join(pkg_path, "meshes", "mock_tool.STL")

        mesh = self.create_mesh(mesh_path)

        if mesh is None:
            return

        co.meshes.append(mesh)
        co.mesh_poses.append(self.relative_pose().pose)
        co.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = "tool_mount_link"
        # aco.object.id = "gripper"
        aco.object = co
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ["tool_mount_link","gripper_base","gripper","screwdriver_base"]

        ps.robot_state.attached_collision_objects.append(aco)

        req = ApplyPlanningScene.Request()
        req.scene = ps

        future = self.scene_client.call_async(req)
        future.add_done_callback(self.attach_done_cb)

    def attach_done_cb(self, future):
        if not self.goal_active:
            return
        self.state = "LOCK"
        self.send_tool_command(1)

    def detach_gripper(self):
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True

        # Remove attached object
        aco = AttachedCollisionObject()
        aco.object.id = "gripper"
        aco.object.operation = CollisionObject.REMOVE

        ps.robot_state.attached_collision_objects.append(aco)

        req = ApplyPlanningScene.Request()
        req.scene = ps

        future = self.scene_client.call_async(req)
        future.add_done_callback(self.detach_done_cb)

    def detach_done_cb(self, future):
        if not self.goal_active:
            return
        self.state = "DETACH_LOCK"
        self.send_tool_command(2)

    def finish_success(self):

        if not self.goal_active:
            return

        self.state = "IDLE"
        self.current_tool = "gripper"

        self._result = ChangeTool.Result()
        self._result.success = True
        self._result.message = "Gripper attached"

        self.goal_handle.succeed()
        self.goal_active = False 
    
    def finish_detach_success(self):

        if not self.goal_active:
            return

        self.state = "IDLE"
        self.current_tool = None

        self._result = ChangeTool.Result()
        self._result.success = True
        self._result.message = "Tool detached"

        self.goal_handle.succeed()
        self.goal_active = False 

    def abort(self, message):
        if not self.goal_active:
            return

        self.get_logger().error(message)
        self.state = "IDLE"

        self._result = ChangeTool.Result()
        self._result.success = False
        self._result.message = message

        self.goal_handle.abort()
        self.goal_active = False

    # ==========================================================
    # HELPERS
    # ==========================================================
    def get_dock_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = 0.193
        pose.pose.position.y = -0.287
        pose.pose.position.z = 0.238
        pose.pose.orientation.x = 1.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose
    
    def get_transform(self, target_frame, source_frame, timeout=2.0):
        """Get transform between frames with error handling"""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, now, rclpy.duration.Duration(seconds=timeout))
           
            t = transform.transform.translation
            r = transform.transform.rotation

            self.get_logger().info(
                f"TF {source_frame} -> {target_frame} | "
                f"Translation: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f} | "
                f"Rotation (quat): x={r.x:.4f}, y={r.y:.4f}, z={r.z:.4f}, w={r.w:.4f}"
            )
            pose = PoseStamped()
            pose.header.frame_id = target_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x 
            pose.pose.position.y = transform.transform.translation.y 
            pose.pose.position.z = transform.transform.translation.z + 0.018
            pose.pose.orientation.x = 1.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            
            return pose
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {str(e)}")
            return None

    def offset_pose(self, base, dx=0, dy=0, dz=0):
        pose = PoseStamped()
        pose.header.frame_id = base.header.frame_id
        pose.pose.position.x = base.pose.position.x + dx
        pose.pose.position.y = base.pose.position.y + dy
        pose.pose.position.z = base.pose.position.z + dz
        pose.pose.orientation = base.pose.orientation
        return pose

    def relative_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "tool_mount_link"
        pose.pose.position.z = 0.01
        pose.pose.orientation.w = 1.0
        return pose

    def create_goal(self, pose):
        pos = PositionConstraint()
        pos.header.frame_id = pose.header.frame_id
        pos.link_name = "tool_mount_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]

        pos.constraint_region.primitives.append(primitive)
        pos.constraint_region.primitive_poses.append(pose.pose)
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = pose.header.frame_id
        ori.link_name = "tool_mount_link"
        ori.orientation = pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.3
        ori.absolute_y_axis_tolerance = 0.3
        ori.absolute_z_axis_tolerance = 0.3
        ori.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)

        goal = MoveGroup.Goal()
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = 0.0
        goal.request.workspace_parameters.max_corner.x = 1.0
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 1.5
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.goal_constraints.append(constraints)
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 1.0
        goal.request.max_acceleration_scaling_factor = 1.0
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = False
        goal.request.start_state = self.get_current_robot_state()

        return goal
    
    def get_current_robot_state(self):
        state = RobotState()
        # Fill with current joint positions or leave empty if you want MoveIt to auto-fill
        state.is_diff = True
        return state


def main(args=None):
    rclpy.init(args=args)
    node = ToolChangeManager()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print("Terminating Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
