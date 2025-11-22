# ros_node.py - 다중 차량 제어를 위한 버전
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from std_msgs.msg import Header, String
from builtin_interfaces.msg import Time
from tier4_system_msgs.srv import ChangeOperationMode
import math
import json
import asyncio

class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_bridge_node')
        
        # 차량별 Publishers 저장소
        self.vehicle_publishers = {}  # vehicle_id: {initialpose: pub, goal: pub, mode_client: client}
        
        self.get_logger().info('ROS Bridge Node initialized for multi-vehicle management')
        
    def _get_vehicle_publishers(self, vehicle_id):
        """차량별 Publisher와 Service Client 가져오기 (없으면 생성)"""
        if vehicle_id not in self.vehicle_publishers:
            # 차량별 토픽 생성 (suffix 붙임)
            initialpose_topic = f'/{vehicle_id}/initialpose'
            goal_topic = f'/{vehicle_id}/planning/mission_planning/goal'
            mode_service = f'/{vehicle_id}/system/operation_mode/change_operation_mode'
            
            self.vehicle_publishers[vehicle_id] = {
                'initialpose': self.create_publisher(
                    PoseWithCovarianceStamped, initialpose_topic, 10
                ),
                'goal': self.create_publisher(
                    PoseStamped, goal_topic, 10
                ),
                'mode_client': self.create_client(
                    ChangeOperationMode, mode_service
                )
            }
            
            self.get_logger().info(f'Created publishers for vehicle {vehicle_id}')
            self.get_logger().info(f'  - InitialPose: {initialpose_topic}')
            self.get_logger().info(f'  - Goal: {goal_topic}')
            self.get_logger().info(f'  - Mode Service: {mode_service}')
        
        return self.vehicle_publishers[vehicle_id]

    def publish_initialpose(self, vehicle_id, x, y, yaw, frame_id="map"):
        """특정 차량의 초기 위치 설정"""
        try:
            pubs = self._get_vehicle_publishers(vehicle_id)
            
            msg = PoseWithCovarianceStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            
            msg.pose.pose.position = Point()
            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.position.z = 0.0
            
            msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)
            
            # Covariance 설정 (불확실성)
            msg.pose.covariance = [0.0] * 36
            msg.pose.covariance[0] = 0.25    # x position variance
            msg.pose.covariance[7] = 0.25    # y position variance
            msg.pose.covariance[35] = 0.06854 # yaw variance
            
            pubs['initialpose'].publish(msg)
            self.get_logger().info(f"Published initialpose for {vehicle_id}: x={x}, y={y}, yaw={yaw}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish initialpose for {vehicle_id}: {e}")

    def publish_goal(self, vehicle_id, x, y, yaw, frame_id="map"):
        """특정 차량의 목표 지점 설정"""
        try:
            pubs = self._get_vehicle_publishers(vehicle_id)
            
            msg = PoseStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            
            msg.pose.position = Point()
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = 0.0
            
            msg.pose.orientation = self._yaw_to_quaternion(yaw)
            
            pubs['goal'].publish(msg)
            self.get_logger().info(f"Published goal for {vehicle_id}: x={x}, y={y}, yaw={yaw}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish goal for {vehicle_id}: {e}")

    async def change_operation_mode(self, vehicle_id, mode):
        """특정 차량의 동작 모드 변경 (1: STOP, 2: AUTONOMOUS, 3: LOCAL, 4: REMOTE)"""
        try:
            pubs = self._get_vehicle_publishers(vehicle_id)
            mode_client = pubs['mode_client']
            
            if not mode_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Operation mode service not available for vehicle {vehicle_id}')
                return None
                
            request = ChangeOperationMode.Request()
            request.mode = int(mode)
            
            future = mode_client.call_async(request)
            self.get_logger().info(f"Requested mode change for {vehicle_id}: {mode}")
            return future
            
        except Exception as e:
            self.get_logger().error(f'Service call failed for vehicle {vehicle_id}: {e}')
            return None
        
    def _yaw_to_quaternion(self, yaw):
        """Yaw 각도를 Quaternion으로 변환"""
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion
    
    def get_active_vehicles(self):
        """현재 활성화된 차량 목록 반환"""
        return list(self.vehicle_publishers.keys())
    
    def cleanup_vehicle(self, vehicle_id):
        """차량 관련 리소스 정리"""
        if vehicle_id in self.vehicle_publishers:
            try:
                # Publisher들을 destroy
                pubs = self.vehicle_publishers[vehicle_id]
                self.destroy_publisher(pubs['initialpose'])
                self.destroy_publisher(pubs['goal'])
                self.destroy_client(pubs['mode_client'])
                
                del self.vehicle_publishers[vehicle_id]
                self.get_logger().info(f"Cleaned up resources for vehicle {vehicle_id}")
                
            except Exception as e:
                self.get_logger().error(f"Error cleaning up vehicle {vehicle_id}: {e}")
    
    def cleanup_all_vehicles(self):
        """모든 차량 리소스 정리"""
        vehicles = list(self.vehicle_publishers.keys())
        for vehicle_id in vehicles:
            self.cleanup_vehicle(vehicle_id)