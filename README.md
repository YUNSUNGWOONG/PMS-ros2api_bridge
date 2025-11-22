# PMS-ros2api_bridge - AUTONOMOUSPCC(VTS&PMS)
<table style="width: 100%; border-collapse: collapse;" align="center"  >
        <tr>
            <td style="width: 50%; text-align: center; vertical-align: middle; border: 1px solid #000;">
                <img src="https://github.com/user-attachments/assets/31397a4b-e048-497f-aa87-5425e26e3059" alt="이미지(1)" style="max-width: 100%; height: auto;">
            </td>
        </tr>
    </table>

## *ROS2 ↔ FastAPI bridge for PMS 차량 관제 시스템*

![last-commit](https://img.shields.io/github/last-commit/YUNSUNGWOONG/PMS-ros2api_bridge?style=flat&logo=git&logoColor=white&color=0080ff)
![repo-top-language](https://img.shields.io/github/languages/top/YUNSUNGWOONG/PMS-ros2api_bridge?style=flat&color=0080ff)
![repo-language-count](https://img.shields.io/github/languages/count/YUNSUNGWOONG/PMS-ros2api_bridge?style=flat&color=0080ff)

*Built with the tools and technologies:*  
![Markdown](https://img.shields.io/badge/Markdown-000000.svg?style=flat&logo=Markdown&logoColor=white)
![GNU Bash](https://img.shields.io/badge/GNU%20Bash-4EAA25.svg?style=flat&logo=GNU-Bash&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-2496ED.svg?style=flat&logo=Docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-22314E.svg?style=flat&logo=ROS&logoColor=white)
![FastAPI](https://img.shields.io/badge/FastAPI-009688.svg?style=flat&logo=FastAPI&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB.svg?style=flat&logo=Python&logoColor=white)
![HTML5](https://img.shields.io/badge/HTML5-E34F26.svg?style=flat&logo=HTML5&logoColor=white)
![CSS3](https://img.shields.io/badge/CSS3-1572B6.svg?style=flat&logo=CSS3&logoColor=white)
![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E.svg?style=flat&logo=JavaScript&logoColor=black)
![YAML](https://img.shields.io/badge/YAML-CB171E.svg?style=flat&logo=YAML&logoColor=white)

---

## Table of Contents
- [Overview](#overview)
- [Architecture](#architecture)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Running with Docker](#running-with-docker)
  - [Accessing the Web UI](#accessing-the-web-ui)
- [Project Structure](#project-structure)
- [Development Notes](#development-notes)

---

## Overview
`PMS-ros2api_bridge` 는 **주차 관리 시스템(PMS)** 을 위한 ROS2 기반 서버와  
웹 대시보드를 연결해 주는 브리지 프로젝트입니다.

- 🛰 **ROS2 ↔ FastAPI 브리지**  
  ROS2 토픽/서비스의 정보를 FastAPI 기반 REST API 및 WebSocket 으로 노출합니다.
- 🚗 **실시간 차량 관제 대시보드**  
  총 차량, 주차 중, 빈 자리, 입차 대기 차량 수를 실시간 카드 형태로 시각화합니다.
- 🎥 **실시간 영상 영역**  
  중앙에 영상 스트림을 띄울 수 있는 전용 박스를 제공하여,  
  주차장 카메라/시뮬레이터 영상을 연동할 수 있습니다.
- 📊 **서버 & WebSocket 상태 모니터링**  
  PMS 서버 활성 여부, WebSocket 연결 상태, 마지막 데이터 업데이트 시각을 표시합니다.
- 🧩 **도커 기반 배포**  
  Docker / Docker Compose 로 손쉽게 로컬·WSL 환경에서 구동할 수 있습니다.

---

## Architecture

고수준의 아키텍처는 다음과 같습니다.

1. **ROS2 Bridge Node**
   - ROS2 네트워크에 참여하여 차량/주차장/시스템 정보를 구독 및 필요 시 퍼블리시.
   - 설정 값은 `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` 등 환경 변수로 제어.

2. **FastAPI Server**
   - `uvicorn` 으로 구동되는 ASGI 서버.
   - REST API 로 상태 조회, 제어 명령 래핑.
   - WebSocket 엔드포인트에서 ROS2 이벤트를 실시간 push.

3. **Web Dashboard (HTML/CSS/JS)**
   - 단일 페이지 대시보드 UI.
   - WebSocket 으로 받은 데이터를 DOM 에 반영해 통계 카드/차량 목록/상태 표시.
   - 중앙의 영상 박스는 `<video>`, `<img>` 등으로 교체하여 실제 스트림 연동 가능.

4. **Docker Environment**
   - `docker-compose.yml` 로 ROS2 + FastAPI 브리지 컨테이너 정의.
   - 필요 시 `network_mode: host` 를 사용해 ROS2 discovery 지원.

---

## Getting Started

### Prerequisites

- OS: Linux or WSL2 + Docker Desktop
- **Docker** & **Docker Compose**
- (선택) NVIDIA GPU 사용 시
  - 최신 NVIDIA 드라이버
  - `nvidia-container-toolkit`
- ROS2 는 컨테이너 이미지 내부에 포함되어 있다고 가정합니다.

### Installation

1. **Clone the repository**
   ```sh
   git clone https://github.com/YUNSUNGWOONG/PMS-ros2api_bridge.git
   cd PMS-ros2api_bridge

2. **Set CycloneDDS & ROS environment at the host**<br>
    > 컨테이너와 동일한 환경이어야 통신 가능함<br>
    > (컨테이너의 ROS_DOMAIN_ID는 30, RMW_IMPLEMENTATION은 rmw_cyclonedds_cpp)
   ```sh
   # 호스트에서 실행
   export ROS_DOMAIN_ID=30
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

3. **Clone the repository ＆ Execute the Server(Based on Docker Compose)**<br>
   ```sh
   cd ~
   git clone https://github.com/YUNSUNGWOONG/PMS-ros2api_bridge.git
   cd PMS-ros2api_bridge
   docker compose up --build


3. **Test**<br>
    > Fastapi 엔드포인트에 테스트용 json 전송, 실제로는 node.js가 이 역할을 대신함<br>
    > Initial pose 설정<br>    
   ```sh
   curl -X POST "http://localhost:8001/initialpose" \
     -H "Content-Type: application/json" \
     -d '{"x": -33.086097717285156, "y": 28.541202545166016, "yaw": 1.5708, "frame_id": "map"}'
    ```
    > Goal 설정 <br>  
   ```sh
   curl -X POST "http://localhost:8001/goal" \
     -H "Content-Type: application/json" \
     -d '{"x": -33.186100006103516, "y": 36.441200256347656, "yaw": 1.482, "frame_id": "map"}'
    ```    
    > Autonomous mode로 변경 <br>  
   ```sh
   curl -X POST "http://localhost:8001/operation_mode" \
     -H "Content-Type: application/json" \
     -d '{"mode": 2}'
    ```        
    > 서비스 상태 확인 <br>  
   ```sh
   curl "http://localhost:8001/health"
    ```   

```sh
#호스트에서 fastapi에 요청 들어올때 마다 실시간으로 발사되는 토픽 확인
# 동일 네트워크, 동일 DDS구현체, 동일 ROS_DOMAIN_ID 이면 어디서든지 확인가능
ros2 topic echo /initialpose
ros2 topic echo /planning/mission_planning/goal
```
