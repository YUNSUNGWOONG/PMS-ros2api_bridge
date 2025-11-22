from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
import rclpy
import asyncio
from ros2api_bridge.ros2api_bridge.ros_node import RosBridgeNode
from pydantic import BaseModel
from typing import Optional, Dict, List
import os
import json
import time
from datetime import datetime
import uuid

app = FastAPI(title="PMS Server - Parking Management System", version="3.0.0")

# CORS 미들웨어 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Static files setup
import pathlib
current_dir = pathlib.Path(__file__).parent
static_dir = current_dir / "static"
if static_dir.exists() and static_dir.is_dir():
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
    print(f"✓ Static files mounted from {static_dir}")

# 간단한 차량 관리 시스템
class VehicleManager:
    def __init__(self):
        self.vehicles: Dict[str, dict] = {}
        self.websocket_connections: List[WebSocket] = []
        self.available_parking_spots: List[dict] = []
        self.auto_assignment_enabled = True
        
        # 출구 좌표 설정
        self.exit_coordinates = {
            "x": 5.0,
            "y": 2.0,
            "yaw": 1.57,
            "frame_id": "map"
        }
        
        # 기본 주차 공간들 (VTS에서 업데이트할 때까지 사용)
        self.default_parking_spots = [
            {"spot_id": "spot_1", "coordinates": {"x": 3.0, "y": 1.0, "yaw": 0.0}},
            {"spot_id": "spot_2", "coordinates": {"x": 3.0, "y": 3.0, "yaw": 0.0}},
            {"spot_id": "spot_3", "coordinates": {"x": 3.0, "y": 5.0, "yaw": 0.0}},
            {"spot_id": "spot_4", "coordinates": {"x": 6.0, "y": 1.0, "yaw": 1.57}},
            {"spot_id": "spot_5", "coordinates": {"x": 6.0, "y": 3.0, "yaw": 1.57}},
        ]
        self.available_parking_spots = self.default_parking_spots.copy()
        
        # 주차장 현황 통계
        self.parking_status = {
            "total_spots": len(self.default_parking_spots),
            "occupied_spots": 0,
            "available_spots": len(self.default_parking_spots)
        }
        
    def add_websocket(self, websocket: WebSocket):
        self.websocket_connections.append(websocket)
        
    def remove_websocket(self, websocket: WebSocket):
        if websocket in self.websocket_connections:
            self.websocket_connections.remove(websocket)
    
    async def broadcast_update(self, data: dict):
        """모든 웹소켓 클라이언트에게 업데이트 전송"""
        if not self.websocket_connections:
            return
            
        safe_data = self._make_json_safe(data)
        message = json.dumps(safe_data)
        disconnected = []
        
        for websocket in self.websocket_connections:
            try:
                await websocket.send_text(message)
            except:
                disconnected.append(websocket)
        
        for ws in disconnected:
            self.remove_websocket(ws)
    
    def _make_json_safe(self, obj):
        """JSON 직렬화 가능한 형태로 변환"""
        if isinstance(obj, dict):
            return {k: self._make_json_safe(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [self._make_json_safe(item) for item in obj]
        elif hasattr(obj, '__dict__'):
            return str(obj)
        else:
            return obj
    
    def _update_parking_status(self):
        """주차장 현황 통계 업데이트"""
        occupied = len([v for v in self.vehicles.values() if v["status"] == "parked"])
        self.parking_status = {
            "total_spots": len(self.default_parking_spots),
            "occupied_spots": occupied,
            "available_spots": len(self.available_parking_spots)
        }
    
    async def vehicle_entry(self, vehicle_data: dict, initial_pose: dict):
        """차량 입차 처리"""
        vehicle_id = vehicle_data.get("vehicle_id")
        
        vehicle_info = {
            "id": vehicle_id,
            "plate_number": vehicle_data["plate_number"],
            "model": vehicle_data.get("model", "Unknown"),
            "status": "entered",
            "entry_time": datetime.now().isoformat(),
            "current_x": initial_pose["x"],
            "current_y": initial_pose["y"],
            "current_yaw": initial_pose["yaw"],
            "waiting_for_parking_spot": True
        }
        
        self.vehicles[vehicle_id] = vehicle_info
        
        # 자동 주차 배정 시도
        assigned_spot = None
        if self.auto_assignment_enabled:
            assigned_spot = await self.try_auto_assign_parking(vehicle_id)
        
        self._update_parking_status()
        
        await self.broadcast_update({
            "type": "vehicle_update",
            "action": "entry",
            "vehicle": vehicle_info,
            "initial_pose": initial_pose,
            "assigned_spot": assigned_spot
        })
        
        print(f"✓ Vehicle {vehicle_id} entered at position ({initial_pose['x']}, {initial_pose['y']})")
        return vehicle_info
    
	
    async def try_auto_assign_parking(self, vehicle_id: str):
        """자동 주차 배정 시도 (+ 배정 즉시 goal 퍼블리시)"""
        if not self.available_parking_spots:
            print(f"⚠ No available parking spots for {vehicle_id}")
            return None

        # 첫 번째 사용 가능한 주차 공간 선택
        parking_spot = self.available_parking_spots[0]

        try:
            # 1) 서버 내부 상태 업데이트(배정 처리)
            await self.assign_parking_spot(vehicle_id, parking_spot["coordinates"])

            # 2) (중요) 출차 시 자리 반납을 위해 어떤 자리를 썼는지 기록
            if vehicle_id in self.vehicles:
                self.vehicles[vehicle_id]["assigned_spot"] = parking_spot["spot_id"]

            # 3) 배정된 자리로 즉시 이동 지시 (ROS2 goal 퍼블리시)
            #    frame_id가 좌표에 없으면 "map" 기본값 사용
            coords = parking_spot["coordinates"]
            ros_node.publish_goal(
                vehicle_id,
                float(coords["x"]),
                float(coords["y"]),
                float(coords["yaw"]),
                coords.get("frame_id", "map")
            )

            # (선택) 바로 자율주행 모드로 전환하고 싶다면 주석 해제
            asyncio.create_task(ros_node.change_operation_mode(vehicle_id, 2))  # 2 = AUTONOMOUS

            # 4) 배정된 주차 공간을 가용 목록에서 제거
            self.available_parking_spots.remove(parking_spot)

            print(f"🎯 Auto-assigned {parking_spot['spot_id']} to {vehicle_id} and published goal")
            return {
                "id": parking_spot["spot_id"],
                "coordinates": coords
            }

        except Exception as e:
            print(f"❌ Auto parking assignment failed: {e}")
            return None

    
    async def assign_parking_spot(self, vehicle_id: str, parking_coordinates: dict):
        """주차 자리 배정"""
        if vehicle_id not in self.vehicles:
            return False
            
        vehicle = self.vehicles[vehicle_id]
        vehicle["status"] = "moving_to_parking"
        vehicle["target_x"] = parking_coordinates["x"]
        vehicle["target_y"] = parking_coordinates["y"]
        vehicle["target_yaw"] = parking_coordinates["yaw"]
        vehicle["waiting_for_parking_spot"] = False
        vehicle["parking_assigned_time"] = datetime.now().isoformat()
        
        await self.broadcast_update({
            "type": "vehicle_update",
            "action": "parking_assigned",
            "vehicle": vehicle,
            "parking_coordinates": parking_coordinates
        })
        
        print(f"✓ Vehicle {vehicle_id} assigned parking spot at ({parking_coordinates['x']}, {parking_coordinates['y']})")
        return True
    
    async def vehicle_parked(self, vehicle_id: str):
        """차량 주차 완료 처리"""
        if vehicle_id not in self.vehicles:
            return False
            
        vehicle = self.vehicles[vehicle_id]
        vehicle["status"] = "parked"
        vehicle["parked_time"] = datetime.now().isoformat()
        
        self._update_parking_status()
        
        await self.broadcast_update({
            "type": "vehicle_update",
            "action": "parked",
            "vehicle": vehicle
        })
        
        print(f"✓ Vehicle {vehicle_id} parked successfully")
        return True
    
    async def vehicle_exit_request(self, vehicle_id: str):
        """차량 출차 요청 처리"""
        if vehicle_id not in self.vehicles:
            return False
            
        vehicle = self.vehicles[vehicle_id]
        vehicle["status"] = "exit_request"
        vehicle["exit_request_time"] = datetime.now().isoformat()
        
        await self.broadcast_update({
            "type": "vehicle_update",
            "action": "exit_request",
            "vehicle": vehicle
        })
        
        print(f"✓ Vehicle {vehicle_id} requested exit")
        return True
    
    async def vehicle_exit_complete(self, vehicle_id: str):
        """차량 출차 완료 처리"""
        if vehicle_id not in self.vehicles:
            return False
        
        vehicle = self.vehicles[vehicle_id]
        vehicle["exit_time"] = datetime.now().isoformat()
        
        # 주차 공간 해제 - 차량이 사용했던 주차 공간을 다시 사용 가능하게 만듦
        if "assigned_spot" in vehicle:
            released_spot = {
                "spot_id": vehicle["assigned_spot"],
                "coordinates": {
                    "x": vehicle.get("target_x"),
                    "y": vehicle.get("target_y"), 
                    "yaw": vehicle.get("target_yaw")
                }
            }
            # 중복 방지하고 다시 추가
            if not any(spot["spot_id"] == released_spot["spot_id"] for spot in self.available_parking_spots):
                self.available_parking_spots.append(released_spot)
                print(f"🔓 Released parking spot {released_spot['spot_id']}")
        
        self._update_parking_status()
        
        await self.broadcast_update({
            "type": "vehicle_update",
            "action": "exit_complete",
            "vehicle": vehicle
        })
        
        # 차량 정보 삭제
        del self.vehicles[vehicle_id]
        
        print(f"✓ Vehicle {vehicle_id} exit completed")
        return True
    
    async def update_available_parking_spots(self, spots_data: dict):
        """VTS에서 받은 주차 공간 정보 업데이트"""
        if spots_data["type"] == "parking_spot_update":
            # 개별 주차 공간 업데이트
            for spot in spots_data["spots"]:
                existing_spot = next(
                    (s for s in self.available_parking_spots if s["spot_id"] == spot["spot_id"]), 
                    None
                )
                
                if spot["occupied"]:
                    # 점유됨 - 사용 가능 목록에서 제거
                    if existing_spot:
                        self.available_parking_spots.remove(existing_spot)
                        print(f"⚠ Parking spot {spot['spot_id']} is now occupied")
                else:
                    # 사용 가능 - 목록에 추가
                    if not existing_spot:
                        self.available_parking_spots.append({
                            "spot_id": spot["spot_id"],
                            "coordinates": spot["coordinates"]
                        })
                        print(f"✅ Parking spot {spot['spot_id']} is now available")
                        
        elif spots_data["type"] == "parking_full_status":
            # 전체 상태 업데이트
            self.available_parking_spots = spots_data["available_spots"]
            print(f"📊 Updated full parking status: {len(self.available_parking_spots)} spots available")
        
        self._update_parking_status()
        
        # 대기 중인 차량들에게 자동 배정 시도
        await self.process_waiting_vehicles()
        
        await self.broadcast_update({
            "type": "parking_spots_update",
            "available_spots": self.available_parking_spots,
            "total_available": len(self.available_parking_spots)
        })
    
    async def process_waiting_vehicles(self):
        """대기 중인 차량들 처리"""
        waiting_vehicles = [
            (vid, v) for vid, v in self.vehicles.items() 
            if v.get("waiting_for_parking_spot", False)
        ]
        
        for vehicle_id, vehicle in waiting_vehicles:
            if await self.try_auto_assign_parking(vehicle_id):
                break  # 한 번에 하나씩만 배정
    
    def get_vehicles_by_status(self, status: str):
        """상태별 차량 목록 반환"""
        # JavaScript의 상태명과 맞춤
        if status == "entered":
            return [v for v in self.vehicles.values() if v["status"] in ["entered", "moving_to_parking"]]
        elif status == "parking":
            return [v for v in self.vehicles.values() if v["status"] == "parked"]
        elif status == "exiting":
            return [v for v in self.vehicles.values() if v["status"] in ["exit_request", "moving_to_exit"]]
        else:
            return [v for v in self.vehicles.values() if v["status"] == status]
    
    def get_all_vehicles(self):
        """모든 차량 정보 반환"""
        return list(self.vehicles.values())
    
    def get_exit_coordinates(self):
        """출구 좌표 반환"""
        return self.exit_coordinates.copy()
    
    def get_parking_status(self):
        """주차장 현황 반환"""
        return self.parking_status.copy()

# 차량 관리자 및 ROS2 노드 초기화
vehicle_manager = VehicleManager()
rclpy.init()
ros_node = RosBridgeNode()

def ros_spin():
    rclpy.spin(ros_node)

threading.Thread(target=ros_spin, daemon=True).start()

# Pydantic 모델들
class VehicleEntryPayload(BaseModel):
    vehicle_id: str
    plate_number: str
    model: Optional[str] = "Unknown Vehicle"
    initial_x: float
    initial_y: float
    initial_yaw: float
    frame_id: Optional[str] = "map"

class ParkingAssignmentPayload(BaseModel):
    vehicle_id: str
    parking_x: float
    parking_y: float
    parking_yaw: float
    frame_id: Optional[str] = "map"

class VehicleStatusPayload(BaseModel):
    vehicle_id: str
    status: str

class GoalPayload(BaseModel):
    vehicle_id: str
    x: float
    y: float
    yaw: float
    frame_id: Optional[str] = "map"

# WebSocket 엔드포인트
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    vehicle_manager.add_websocket(websocket)
    
    try:
        initial_data = {
            "type": "initial_data",
            "vehicles": vehicle_manager.get_all_vehicles(),
            "available_spots": vehicle_manager.available_parking_spots,
            "exit_coordinates": vehicle_manager.get_exit_coordinates(),
            "parking_status": vehicle_manager.get_parking_status()
        }
        
        safe_data = vehicle_manager._make_json_safe(initial_data)
        await websocket.send_text(json.dumps(safe_data))
        
        while True:
            data = await websocket.receive_text()
            
    except WebSocketDisconnect:
        vehicle_manager.remove_websocket(websocket)

# 메인 페이지
@app.get("/")
async def serve_main_page():
    """메인 페이지로 static/index.html 파일을 제공합니다."""
    index_path = os.path.join(static_dir, "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    else:
        # 혹시 파일이 없을 경우를 대비한 예외 처리
        raise HTTPException(status_code=404, detail="index.html not found")
# 혹시 파일이 없을 경우를 대비한 예외 처리raise HTTPException(status_code=404, detail="index.html not found")
# ======================
# 핵심 API 엔드포인트들
# ======================

@app.post("/vehicle/entry")
async def vehicle_entry(vehicle_data: VehicleEntryPayload):
    """🚗 차량 입차 - 차량에서 직접 호출하는 간단한 버전"""
    try:
        vehicle_id = vehicle_data.vehicle_id
        
        # 1. ROS2 초기위치 발행
        ros_node.publish_initialpose(
            vehicle_id,
            vehicle_data.initial_x, 
            vehicle_data.initial_y, 
            vehicle_data.initial_yaw, 
            vehicle_data.frame_id
        )
        
        # 2. 차량 정보 등록 및 자동 주차 배정
        initial_pose = {
            "x": vehicle_data.initial_x,
            "y": vehicle_data.initial_y,
            "yaw": vehicle_data.initial_yaw,
            "frame_id": vehicle_data.frame_id
        }
        
        vehicle_info = await vehicle_manager.vehicle_entry(vehicle_data.dict(), initial_pose)
        
        return {
            "status": "success",
            "message": f"Vehicle {vehicle_id} entry processed. Auto parking assignment attempted.",
            "data": {
                "vehicle": vehicle_info,
                "available_spots": len(vehicle_manager.available_parking_spots),
                "auto_assigned": not vehicle_info.get("waiting_for_parking_spot", True)
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/vehicle/status")
async def update_vehicle_status(status_data: VehicleStatusPayload):
    """🚗 차량 상태 업데이트 - 차량에서 직접 호출"""
    try:
        vehicle_id = status_data.vehicle_id
        status = status_data.status
        
        success = False
        if status == "parked":
            success = await vehicle_manager.vehicle_parked(vehicle_id)
        elif status == "exit_request":
            success = await vehicle_manager.vehicle_exit_request(vehicle_id)
            
            # 출차 요청 시 자동으로 출구로 안내
            if success:
                exit_coords = vehicle_manager.get_exit_coordinates()
                ros_node.publish_goal(
                    vehicle_id,
                    exit_coords["x"],
                    exit_coords["y"],
                    exit_coords["yaw"],
                    exit_coords["frame_id"]
                )
                
                # 자율주행 모드로 변경
                future = await ros_node.change_operation_mode(vehicle_id, 2)
                
        elif status == "exit_complete":
            success = await vehicle_manager.vehicle_exit_complete(vehicle_id)
        
        if success:
            return {
                "status": "success",
                "message": f"Vehicle status updated to {status}",
                "vehicle_id": vehicle_id,
                "auto_guided_to_exit": status == "exit_request"
            }
        else:
            raise HTTPException(status_code=404, detail="Vehicle not found")
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ======================
# ROS2 관련 API들
# ======================

@app.post("/goal")
async def send_goal(goal_data: GoalPayload):
    """ROS2 목표점 전송 - JavaScript에서 호출"""
    try:
        ros_node.publish_goal(
            goal_data.vehicle_id,
            goal_data.x,
            goal_data.y,
            goal_data.yaw,
            goal_data.frame_id
        )
        
        return {
            "status": "success",
            "message": f"Goal sent to vehicle {goal_data.vehicle_id}",
            "goal": {
                "x": goal_data.x,
                "y": goal_data.y,
                "yaw": goal_data.yaw
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ======================
# VTS 및 관리 API들
# ======================

@app.post("/parking/spots/update")
async def update_parking_spots(spots_data: dict):
    """📹 VTS에서 주차 공간 업데이트 수신"""
    try:
        await vehicle_manager.update_available_parking_spots(spots_data)
        return {
            "status": "success",
            "message": "Parking spots updated",
            "available_count": len(vehicle_manager.available_parking_spots)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/vehicles")
async def get_all_vehicles():
    """모든 차량 정보 조회"""
    vehicles = vehicle_manager.get_all_vehicles()
    return {
        "status": "success",
        "count": len(vehicles),
        "vehicles": vehicles
    }

@app.get("/vehicles/{status}")
async def get_vehicles_by_status(status: str):
    """상태별 차량 목록 조회"""
    vehicles = vehicle_manager.get_vehicles_by_status(status)
    return {
        "status": "success",
        "count": len(vehicles),
        "vehicles": vehicles
    }

@app.get("/parking/spots/available")
async def get_available_parking_spots():
    """현재 사용 가능한 주차 공간 조회"""
    return {
        "status": "success",
        "available_spots": vehicle_manager.available_parking_spots,
        "count": len(vehicle_manager.available_parking_spots),
        "parking_status": vehicle_manager.get_parking_status()
    }

# ======================
# 테스트용 API들
# ======================

@app.post("/test/simulate-vehicle-entry")
async def simulate_vehicle_entry():
    """🧪 테스트용: 가상 차량 입차 시뮬레이션"""
    import random
    
    vehicle_id = f"test_car_{int(time.time() * 1000) % 10000}"
    
    fake_vehicle_data = VehicleEntryPayload(
        vehicle_id=vehicle_id,
        plate_number=f"{random.randint(10,99)}가{random.randint(1000,9999)}",
        model="Test Vehicle",
        initial_x=random.uniform(-1.0, 1.0),
        initial_y=random.uniform(-1.0, 1.0),
        initial_yaw=0.0
    )
    
    try:
        response = await vehicle_entry(fake_vehicle_data)
        return {
            "status": "success",
            "message": "Simulated vehicle entry",
            "vehicle_data": fake_vehicle_data.dict(),
            "response": response
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/health")
async def health_check():
    """시스템 상태 확인"""
    try:
        return {
            "status": "healthy",
            "message": "Simple PMS Server is running",
            "vehicle_count": len(vehicle_manager.get_all_vehicles()),
            "available_spots": len(vehicle_manager.available_parking_spots),
            "auto_assignment": vehicle_manager.auto_assignment_enabled,
            "parking_status": vehicle_manager.get_parking_status(),
            "timestamp": time.strftime('%Y-%m-%d %H:%M:%S')
        }
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Service unhealthy: {str(e)}")

@app.on_event("shutdown")
def shutdown_event():
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    print("=" * 60)
    print("🚗 SIMPLE PMS SERVER v3.0 - PROTOTYPE")
    print("=" * 60)
    print("✨ 차량에서 직접 POST 요청만 보내면 됩니다!")
    print()
    print("📝 차량 입차 예시:")
    print('   POST /vehicle/entry')
    print('   {"vehicle_id": "car_001", "plate_number": "12가3456", ...}')
    print()
    print("📝 상태 업데이트 예시:")  
    print('   POST /vehicle/status')
    print('   {"vehicle_id": "car_001", "status": "parked"}')
    print()
    print("🌐 Web interface: http://localhost:8001")
    print("📡 WebSocket: ws://localhost:8001/ws")
    print("🧪 Test endpoint: POST /test/simulate-vehicle-entry")
    print("=" * 60)
    
    uvicorn.run(app, host="0.0.0.0", port=8001)