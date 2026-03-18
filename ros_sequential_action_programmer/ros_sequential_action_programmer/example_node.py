# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionServer, CancelResponse, GoalResponse

# from sensor_msgs.srv import SetCameraInfo
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration

# class CameraAndTrajectoryNode(Node):
#     def __init__(self):
#         super().__init__('camera_and_trajectory_node')

#         # --- Service: SetCameraInfo ---
#         self.srv = self.create_service(
#             SetCameraInfo,
#             'set_camera_info',
#             self.set_camera_info_callback
#         )
#         self.get_logger().info('Service "set_camera_info" ready.')

#         # --- Action Server: FollowJointTrajectory ---
#         self._action_server = ActionServer(
#             self,
#             FollowJointTrajectory,
#             'follow_joint_trajectory',
#             execute_callback=self.execute_callback,
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback
#         )
#         self.get_logger().info('Action server "follow_joint_trajectory" ready.')

#         # Internal storage of camera info
#         self._camera_info = None

#     # -------- Service Callback --------
#     def set_camera_info_callback(self, request, response):
#         # Store the provided camera info internally
#         self._camera_info = request.camera_info
#         self.get_logger().info(f"Received camera info: {self._camera_info.width}x{self._camera_info.height}")
#         response.success = True
#         response.status_message = 'Camera info updated successfully.'
#         return response

#     # -------- Action Callbacks --------
#     def goal_callback(self, goal_request):
#         self.get_logger().info('Received FollowJointTrajectory goal request.')
#         # Accept all goals for demonstration
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().info('Cancel request received.')
#         return CancelResponse.ACCEPT

#     async def execute_callback(self, goal_handle):
#         self.get_logger().info('Executing joint trajectory...')
#         trajectory = goal_handle.request.trajectory

#         feedback_msg = FollowJointTrajectory.Feedback()
#         for point_idx, point in enumerate(trajectory.points):
#             # Simulate processing each point
#             self.get_logger().info(
#                 f"Moving to point {point_idx}: positions={point.positions}"
#             )
#             feedback_msg.actual.positions = point.positions
#             feedback_msg.desired = point
#             feedback_msg.error.positions = [
#                 0.0 for _ in point.positions
#             ]  # No error in this demo
#             goal_handle.publish_feedback(feedback_msg)

#             # Simulate time delay (use point.time_from_start if set)
#             delay = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
#             await self._sleep(delay if delay > 0 else 0.5)

#         goal_handle.succeed()

#         result = FollowJointTrajectory.Result()
#         result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
#         self.get_logger().info('Trajectory execution completed.')
#         return result

#     async def _sleep(self, seconds):
#         # Non-blocking sleep using ROS 2 executor
#         timer = self.create_timer(seconds, lambda: None)
#         await self.executor.create_task(timer)
#         timer.cancel()


# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraAndTrajectoryNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from diagnostic_msgs.srv import AddDiagnostics
from diagnostic_msgs.msg import KeyValue
from std_srvs.srv import SetBool
from example_interfaces.action import Fibonacci
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import OccupancyGrid, MapMetaData
from builtin_interfaces.msg import Time


from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class SimpleServiceAndActionNode(Node):
    def __init__(self):
        super().__init__('simple_service_and_action_node')

        # ---- Service 1: SetBool ----
        self._flag = False
        self._srv_toggle = self.create_service(
            SetBool,
            'toggle_flag',
            self.toggle_flag_callback
        )
        self.get_logger().info('Service "toggle_flag" ready.')

        # ---- Service 2: GetPlan ----
        self._srv_plan = self.create_service(
            GetPlan,
            'get_plan_demo',
            self.get_plan_callback
        )
        self.get_logger().info('Service "get_plan_demo" ready.')

        # ---- Service 3: GetMap (returns array data) ----
        self._srv_map = self.create_service(
            GetMap,
            'get_map_demo',
            self.get_map_callback
        )
        self.get_logger().info('Service "get_map_demo" ready.')

        # ---- Action: Fibonacci ----
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_fibonacci,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Action server "fibonacci" ready.')

        self.srv = self.create_service(
            AddDiagnostics,
            'add_diagnostics_demo',
            self.handle_add_diag
        )
        
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            'follow_joint_trajectory',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )
        
        
    # ---------- Service callbacks ----------
    def toggle_flag_callback(self, request, response):
        self._flag = request.data
        response.success = True
        response.message = f'Flag is now {"ON" if self._flag else "OFF"}'
        self.get_logger().info(response.message)
        return response

    def get_plan_callback(self, request, response):
        from nav_msgs.msg import Path
        path = Path()
        path.header.frame_id = request.start.header.frame_id
        path.poses = [request.start, request.goal]
        response.plan = path
        return response

    def get_map_callback(self, request, response):
        """
        Dummy implementation of GetMap.
        Returns a 10x10 occupancy grid with a simple gradient pattern.
        """
        grid = OccupancyGrid()

        # Header
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()

        # Map info
        info = MapMetaData()
        info.width = 10
        info.height = 10
        info.resolution = 0.5  # meters/cell
        info.origin.position.x = 0.0
        info.origin.position.y = 0.0
        info.origin.position.z = 0.0
        grid.info = info

        # Fill with a gradient from 0 to 100
        grid.data = [int((x + y) % 100) for y in range(10) for x in range(10)]

        response.map = grid
        self.get_logger().info("Sent dummy occupancy grid with 100 cells.")
        return response

    # ---------- Action callbacks ----------
    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received Fibonacci goal: {goal_request.order}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received for Fibonacci action')
        return CancelResponse.ACCEPT

    async def execute_fibonacci(self, goal_handle):
        order = goal_handle.request.order
        self.get_logger().info(f'Executing Fibonacci of order {order}')

        feedback = Fibonacci.Feedback()
        result = Fibonacci.Result()

        sequence = [0, 1]
        feedback.sequence = sequence.copy()
        goal_handle.publish_feedback(feedback)

        for i in range(2, order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Fibonacci goal canceled')
                return Fibonacci.Result()

            sequence.append(sequence[i-1] + sequence[i-2])
            feedback.sequence = sequence.copy()
            goal_handle.publish_feedback(feedback)
            await self._sleep(0.5)

        result.sequence = sequence
        goal_handle.succeed()
        self.get_logger().info(f'Fibonacci result: {sequence}')
        return result

    async def _sleep(self, seconds):
        done = rclpy.task.Future()
        timer = self.create_timer(seconds, lambda: done.set_result(True))
        await done
        timer.cancel()


    def goal_cb(self, goal_request):
        # goal_request is FollowJointTrajectory.Goal
        self.get_logger().info(
            f"Received goal with joints: {list(goal_request.trajectory.joint_names)} "
            f"and {len(goal_request.trajectory.points)} trajectory points"
        )
        # Always accept in this demo
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel request received.")
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        """Pretend to follow the trajectory and immediately succeed."""
        goal = goal_handle.request

        # Publish a dummy feedback for each point
        feedback = FollowJointTrajectory.Feedback()
        for i, point in enumerate(goal.trajectory.points):
            feedback.joint_names = goal.trajectory.joint_names
            feedback.desired = point  # Just echo the point
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f"Feedback for point {i}: positions={list(point.positions)}"
            )

        # Result
        result = FollowJointTrajectory.Result()
        result.error_code = 0  # SUCCESS in control_msgs
        goal_handle.succeed()
        self.get_logger().info("Trajectory execution finished.")
        return result
    
    def handle_add_diag(self, request, response):
        self.get_logger().info(f"Namespace: {request.load_namespace}")
        for kv in request.kvs:
            self.get_logger().info(f"  {kv.key}: {kv.value}")
        response.success = True
        response.message = "Received diagnostics"
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceAndActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
