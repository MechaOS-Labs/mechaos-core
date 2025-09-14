#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mechaos_msgs.msg import Task, TaskCompletion
import json
import hashlib

class TaskExecutor(Node):
    def __init__(self):
        super().__init__('mechaos_task_executor')

        self.task_subscription = self.create_subscription(
            Task,
            '/mechaos/tasks',
            self.handle_task,
            10
        )

        self.completion_publisher = self.create_publisher(
            TaskCompletion,
            '/mechaos/task_completions',
            10
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.capabilities = ['navigation', 'manipulation']

    def handle_task(self, msg):
        """Process incoming task assignment"""
        self.get_logger().info(f'Received task: {msg.task_id}')

        if msg.task_type in self.capabilities:
            self.execute_task(msg)
        else:
            self.get_logger().warn(f'Cannot handle task type: {msg.task_type}')

    def execute_task(self, task_msg):
        """Execute the assigned task"""
        try:
            task_data = {
                'id': task_msg.task_id,
                'type': task_msg.task_type,
                'location': task_msg.location
            }

            if task_data['type'] == 'delivery':
                success = self.execute_delivery(task_data)
            elif task_data['type'] == 'inspection':
                success = self.execute_inspection(task_data)
            else:
                success = False

            self.report_completion(task_msg, success)
            
        except Exception as e:
            self.get_logger().error(f'Task execution failed: {str(e)}')
            self.report_completion(task_msg, False, str(e))

    def execute_delivery(self, task_data):
        """Execute delivery task"""
        self.get_logger().info(f'Executing delivery to {task_data["location"]}')

        import time
        time.sleep(5)  # Simulate work

        return True

    def report_completion(self, original_task, success, error_msg=""):
        """Report task completion to the network"""
        completion_msg = TaskCompletion()
        completion_msg.task_id = original_task.task_id
        completion_msg.robot_id = self.robot_id
        completion_msg.completion_time = int(time.time())
        completion_msg.success = success
        completion_msg.error_message = error_msg
        
        proof_data = {
            'task_id': original_task.task_id,
            'robot_id': self.robot_id,
            'completion_time': completion_msg.completion_time,
            'success': success
        }
        completion_msg.proof_hash = hashlib.sha256(
            json.dumps(proof_data).encode()
        ).hexdigest()
        
        self.completion_publisher.publish(completion_msg)
        self.get_logger().info(f'Task {original_task.task_id} completed: {success}')

def main(args=None):
    rclpy.init(args=args)
    executor = TaskExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()