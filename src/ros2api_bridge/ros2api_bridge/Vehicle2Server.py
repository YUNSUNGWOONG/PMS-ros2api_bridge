# vehicle_ros2_node.py - 차량에서 실행되는 ROS2 노드

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import requests
from threading import Thread

# 커스텀 메시지 타입 (실제로는 별도 패키지에서 정의)
class VehicleStatusMsg:
    def __init__(self):
        self.vehicle_id = ""
        self.plate_number = ""
        self.model = ""
        self.status = ""  # "entry", "parked", "exit_request", "exit_complete"
        self.timestamp = 0

class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')
        
        # 서버 설정
        self.server_url = "http://localhost:8001"
        self.vehicle_id = "V_TEST_001"
        self.plate_number = "12가3456"
        self.model = "Genesis GV70"
        
        # Publishers - 서버로 상태 전송용
        self.status_publisher = self.create_publisher(
            String, '/vehicle/status_update', 10
        )
        
        # Subscribers - 서버로부터 명령 수신용  
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/vehicle/navigation_goal',
            self.goal_callback,
            10
        )
        
        # HTTP 클라이언트 스레드
        self.http_thread = Thread(target=self.http_communication_loop, daemon=True)
        self.http_thread.start()
        
        self.get_logger().info(f'Vehicle Node Started: {self.vehicle_id}')
    
    def send_entry_signal(self):
        """입차 신호 전송 (ROS2 + HTTP)"""
        # ROS2 토픽으로 발행
        msg = String()
        status_data = {
            "vehicle_id": self.vehicle_id,
            "plate_number": self.plate_number,
            "model": self.model,
            "status": "entry",
            "timestamp": self.get_clock().now().nanoseconds
        }
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        
        # HTTP로도 전송 (백업)
        self.send_http_entry()
        
        self.get_logger().info('Entry signal sent')
    
    def send_parking_complete(self):
        """주차 완료 신호 전송"""
        self.send_status_update("parked")
        
    def send_exit_request(self):
        """출차 요청 신호 전송"""  
        self.send_status_update("exit_request")
        
    def send_exit_complete(self):
        """출차 완료 신호 전송"""
        self.send_status_update("exit_complete")
    
    def send_status_update(self, status):
        """상태 업데이트 공통 함수"""
        # ROS2 토픽으로 발행
        msg = String()
        status_data = {
            "vehicle_id": self.vehicle_id,
            "status": status,
            "timestamp": self.get_clock().now().nanoseconds
        }
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        
        # HTTP로도 전송
        self.send_http_status(status)
        
        self.get_logger().info(f'Status update sent: {status}')
    
    def goal_callback(self, msg):
        """서버로부터 목표 지점 수신"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        goal_yaw = self.quaternion_to_yaw(msg.pose.orientation)
        
        self.get_logger().info(f'Received navigation goal: x={goal_x}, y={goal_y}, yaw={goal_yaw}')
        
        # 실제 차량 제어 시스템에 명령 전달
        self.navigate_to_goal(goal_x, goal_y, goal_yaw)
    
    def navigate_to_goal(self, x, y, yaw):
        """실제 차량 자율주행 시스템에 목표 지점 전달"""
        # 여기서 실제 Autoware나 다른 자율주행 스택과 연동
        # 예시: Autoware의 /planning/mission_planning/goal 토픽으로 전달
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = self.yaw_to_quaternion(yaw)
        
        # Autoware 목표 지점 퍼블리셔 (실제 구현시)
        # self.autoware_goal_publisher.publish(goal_msg)
        
        self.get_logger().info(f'Navigating to goal: ({x}, {y}, {yaw})')
        
        # 시뮬레이션: 3초 후 주차 완료 신호
        self.create_timer(3.0, self.simulate_parking_complete)
    
    def simulate_parking_complete(self):
        """주차 완료 시뮬레이션"""
        self.send_parking_complete()
    
    # HTTP 통신 관련 메소드들
    def send_http_entry(self):
        """HTTP로 입차 신호 전송"""
        try:
            payload = {
                "vehicle_id": self.vehicle_id,
                "plate_number": self.plate_number,
                "model": self.model
            }
            response = requests.post(
                f"{self.server_url}/vehicle/entry",
                json=payload,
                timeout=5
            )
            if response.status_code == 200:
                self.get_logger().info("HTTP entry signal sent successfully")
            else:
                self.get_logger().error(f"HTTP entry failed: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"HTTP entry error: {e}")
    
    def send_http_status(self, status):
        """HTTP로 상태 업데이트 전송"""
        try:
            payload = {
                "vehicle_id": self.vehicle_id,
                "status": status
            }
            response = requests.post(
                f"{self.server_url}/vehicle/status",
                json=payload,
                timeout=5
            )
            if response.status_code == 200:
                self.get_logger().info(f"HTTP status update sent: {status}")
            else:
                self.get_logger().error(f"HTTP status update failed: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"HTTP status update error: {e}")
    
    def http_communication_loop(self):
        """HTTP 통신 루프"""
        # 주기적으로 서버 상태 확인 등
        pass
    
    # 유틸리티 함수들
    def quaternion_to_yaw(self, quaternion):
        """Quaternion을 yaw 각도로 변환"""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        """yaw 각도를 Quaternion으로 변환"""
        import math
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    
    vehicle_node = VehicleNode()
    
    try:
        # 시뮬레이션: 2초 후 입차 신호 전송
        timer = vehicle_node.create_timer(2.0, vehicle_node.send_entry_signal)
        
        rclpy.spin(vehicle_node)
    except KeyboardInterrupt:
        pass
    finally:
        vehicle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 사용 방법:
# ros2 run your_package vehicle_ros2_node