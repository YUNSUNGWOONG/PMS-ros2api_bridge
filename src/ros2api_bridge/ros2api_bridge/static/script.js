// script.js - Enhanced PMS Server JavaScript with WebSocket

const API_BASE = 'http://localhost:8001';
const WS_URL = 'ws://localhost:8001/ws';
let currentTab = 'entered';
let websocket = null;
let vehicles = {
    entered: [],
    parking: [],
    exiting: []
};
let parkingStatus = {
    total_spots: 0,
    occupied_spots: 0,
    available_spots: 0
};

// 초기화
document.addEventListener('DOMContentLoaded', function() {
    connectWebSocket();
    setupEventListeners();
    updateSystemInfo();
});

// WebSocket 연결
function connectWebSocket() {
    try {
        websocket = new WebSocket(WS_URL);
        
        websocket.onopen = function(event) {
            console.log('✓ WebSocket connected');
            updateConnectionStatus(true);
        };
        
        websocket.onmessage = function(event) {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        };
        
        websocket.onclose = function(event) {
            console.log('✗ WebSocket disconnected');
            updateConnectionStatus(false);
            // 재연결 시도
            setTimeout(connectWebSocket, 5000);
        };
        
        websocket.onerror = function(error) {
            console.error('WebSocket error:', error);
            updateConnectionStatus(false);
        };
        
    } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        updateConnectionStatus(false);
        setTimeout(connectWebSocket, 5000);
    }
}

// WebSocket 메시지 처리
function handleWebSocketMessage(data) {
    console.log('Received WebSocket message:', data);
    
    switch (data.type) {
        case 'initial_data':
            updateVehicleData(data.vehicles);
            if (data.parking_status) {
                updateParkingStatus(data.parking_status);
            }
            displayVehicles();
            updateSystemInfo();
            break;
            
        case 'vehicle_update':
            handleVehicleUpdate(data);
            break;
            
        case 'parking_spots_update':
            if (data.parking_status) {
                updateParkingStatus(data.parking_status);
            }
            updateSystemInfo();
            break;
            
        default:
            console.log('Unknown message type:', data.type);
    }
}

// 차량 업데이트 처리
function handleVehicleUpdate(data) {
    const { action, vehicle, assigned_spot } = data;
    
    switch (action) {
        case 'entry':
            showNotification(`차량 입차: ${vehicle.plate_number}`, 'success');
            if (assigned_spot) {
                showNotification(`주차구역 자동 배정: ${assigned_spot.id}`, 'info');
            }
            break;
            
        case 'parking_assigned':
            showNotification(`주차구역 배정: ${vehicle.plate_number}`, 'info');
            break;
            
        case 'parked':
            showNotification(`주차 완료: ${vehicle.plate_number}`, 'success');
            break;
            
        case 'exit_request':
            showNotification(`출차 요청: ${vehicle.plate_number}`, 'warning');
            break;
            
        case 'exit_complete':
            showNotification(`출차 완료: ${vehicle.plate_number}`, 'success');
            break;
    }
    
    // 차량 데이터 새로고침
    loadVehicleData();
}

// 차량 데이터 업데이트
function updateVehicleData(vehicleList) {
    // 상태별로 차량 분류 (서버의 get_vehicles_by_status와 일치하도록)
    vehicles = {
        entered: vehicleList.filter(v => ['entered', 'moving_to_parking'].includes(v.status)),
        parking: vehicleList.filter(v => v.status === 'parked'),
        exiting: vehicleList.filter(v => ['exit_request', 'moving_to_exit'].includes(v.status))
    };
}

// 주차장 현황 업데이트
function updateParkingStatus(status) {
    parkingStatus = status;
}

// 이벤트 리스너 설정
function setupEventListeners() {
    // 탭 전환 이벤트
    document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', function() {
            const tabName = this.textContent.trim();
            let status = 'entered';
            if (tabName === '주차중') status = 'parking';
            else if (tabName === '출차') status = 'exiting';
            
            switchTab(status);
        });
    });
}

// 연결 상태 업데이트
function updateConnectionStatus(connected) {
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.status-indicator span');
    
    if (statusDot && statusText) {
        if (connected) {
            statusDot.className = 'status-dot connected';
            statusText.textContent = '실시간 연결';
        } else {
            statusDot.className = 'status-dot disconnected';
            statusText.textContent = '연결 끊김';
        }
    }
}

// 시스템 정보 업데이트
function updateSystemInfo() {
    const systemInfo = document.getElementById('systemInfo');
    if (!systemInfo) return;
    
    const totalVehicles = Object.values(vehicles).flat().length;
    
    systemInfo.innerHTML = `
        <!-- 왼쪽 : 총 차량 / 주차중 / 빈 자리 / 입차 대기 카드 -->
        <div class="system-stats">
            <div class="stat-card">
                <div class="stat-number">${totalVehicles}</div>
                <div class="stat-label">총 차량</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">${parkingStatus.occupied_spots || 0}</div>
                <div class="stat-label">주차중</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">${parkingStatus.available_spots || 0}</div>
                <div class="stat-label">빈 자리</div>
            </div>
            <div class="stat-card">
                <div class="stat-number">${vehicles.entered?.length || 0}</div>
                <div class="stat-label">입차 대기</div>
            </div>
        </div>

        <!-- 가운데 : 실시간 영상 영역 -->
        <div class="video-box">
            <!-- 실제 스트림을 <img>나 <video>로 바꿔 끼우면 됨 -->
            <!-- 예: <video id="liveVideo" autoplay muted></video> -->
            <div class="video-placeholder">
                <span>실시간 영상 영역</span>
            </div>
        </div>
        
        <!-- 오른쪽 : PMS 서버 상태 -->
        <div class="system-info">
            <h3>PMS 서버 상태</h3>
            <div class="info-grid">
                <div class="info-item">
                    <strong>서버:</strong> 
                    <span class="status-active">활성</span>
                </div>
                <div class="info-item">
                    <strong>WebSocket:</strong> 
                    <span class="${websocket && websocket.readyState === WebSocket.OPEN ? 'status-active' : 'status-inactive'}">
                        ${websocket && websocket.readyState === WebSocket.OPEN ? '연결됨' : '연결 안됨'}
                    </span>
                </div>
                <div class="info-item">
                    <strong>마지막 업데이트:</strong> 
                    <span>${new Date().toLocaleTimeString()}</span>
                </div>
            </div>
        </div>
    `;
}

// 탭 전환
function switchTab(tabName) {
    currentTab = tabName;
    
    // 탭 활성화 상태 변경
    document.querySelectorAll('.tab').forEach(tab => {
        tab.classList.remove('active');
    });
    
    // 현재 탭 활성화
    const tabNames = { entered: '입차', parking: '주차중', exiting: '출차' };
    const activeTab = Array.from(document.querySelectorAll('.tab')).find(
        tab => tab.textContent.trim() === tabNames[tabName]
    );
    if (activeTab) {
        activeTab.classList.add('active');
    }
    
    // 차량 데이터 표시
    displayVehicles();
}

// 차량 데이터 로드 (WebSocket 대신 사용)
async function loadVehicleData() {
    try {
        const response = await fetch(`${API_BASE}/vehicles`);
        if (response.ok) {
            const data = await response.json();
            updateVehicleData(data.vehicles);
            displayVehicles();
            updateSystemInfo();
        }
    } catch (error) {
        console.error('Error loading vehicle data:', error);
    }
}

// 차량 목록 표시
function displayVehicles() {
    const vehicleList = document.getElementById('vehicleList');
    const vehicleCount = document.getElementById('vehicleCount');
    if (!vehicleList || !vehicleCount) return;
    
    const currentVehicles = vehicles[currentTab] || [];
    const totalCount = currentVehicles.length;
    
    vehicleCount.textContent = `${totalCount}대`;

    if (totalCount === 0) {
        vehicleList.innerHTML = `
            <div class="empty-state">
                <div style="font-size: 2em; margin-bottom: 10px;">🚗</div>
                <p>${getTabName(currentTab)} 차량이 없습니다</p>
                <small>V2X 시스템을 통해 실시간으로 차량 정보가 업데이트됩니다</small>
            </div>
        `;
        return;
    }

    vehicleList.innerHTML = currentVehicles.map(vehicle => `
        <div class="vehicle-card" data-vehicle-id="${vehicle.id}">
            <div class="vehicle-header">
                <div class="vehicle-id">${vehicle.plate_number}</div>
                <div class="vehicle-status status-${vehicle.status}">
                    ${getStatusText(vehicle.status)}
                </div>
            </div>
            <div class="vehicle-info">
                <div class="info-row">
                    <strong>차종:</strong> ${vehicle.model}
                </div>
                ${vehicle.parking_spot ? `
                <div class="info-row">
                    <strong>주차구역:</strong> ${vehicle.parking_spot}
                </div>` : ''}
                ${vehicle.assigned_spot ? `
                <div class="info-row">
                    <strong>배정구역:</strong> ${vehicle.assigned_spot}
                </div>` : ''}
                <div class="info-row">
                    <strong>시간:</strong> ${getTimeText(vehicle)}
                </div>
                <div class="vehicle-actions">
                    ${generateActionButtons(vehicle)}
                </div>
            </div>
        </div>
    `).join('');
}

// 차량별 액션 버튼 생성
function generateActionButtons(vehicle) {
    const buttons = [];
    
    switch (vehicle.status) {
        case 'entered':
        case 'moving_to_parking':
            if (vehicle.target_x && vehicle.target_y) {
                buttons.push(`
                    <button class="action-btn btn-primary" onclick="sendVehicleToParking('${vehicle.id}')">
                        📍 주차 지시
                    </button>
                `);
            }
            break;
            
        case 'parked':
            buttons.push(`
                <button class="action-btn btn-warning" onclick="requestVehicleExit('${vehicle.id}')">
                    🚪 출차 요청
                </button>
            `);
            break;
            
        case 'exit_request':
        case 'moving_to_exit':
            buttons.push(`
                <button class="action-btn btn-success" onclick="completeVehicleExit('${vehicle.id}')">
                    ✅ 출차 완료
                </button>
            `);
            break;
    }
    
    return buttons.join('');
}

// 차량 액션 함수들
async function sendVehicleToParking(vehicleId) {
    try {
        const vehicle = findVehicleById(vehicleId);
        if (!vehicle || !vehicle.target_x || !vehicle.target_y) {
            showNotification('주차 목표 지점이 설정되지 않았습니다', 'error');
            return;
        }
        
        // ROS2를 통해 목표 지점 전송
        const response = await fetch(`${API_BASE}/goal`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                vehicle_id: vehicleId,
                x: vehicle.target_x,
                y: vehicle.target_y,
                yaw: vehicle.target_yaw || 0.0
            })
        });
        
        if (response.ok) {
            showNotification(`차량 ${vehicle.plate_number}에게 주차 지시를 전송했습니다`, 'success');
        }
        
    } catch (error) {
        console.error('Error sending parking instruction:', error);
        showNotification('주차 지시 전송 실패', 'error');
    }
}

async function requestVehicleExit(vehicleId) {
    try {
        const response = await fetch(`${API_BASE}/vehicle/status`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                vehicle_id: vehicleId,
                status: 'exit_request'
            })
        });
        
        if (response.ok) {
            const vehicle = findVehicleById(vehicleId);
            showNotification(`차량 ${vehicle?.plate_number || vehicleId} 출차 요청 처리됨`, 'success');
        }
    } catch (error) {
        console.error('Error requesting vehicle exit:', error);
        showNotification('출차 요청 실패', 'error');
    }
}

async function completeVehicleExit(vehicleId) {
    try {
        const response = await fetch(`${API_BASE}/vehicle/status`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                vehicle_id: vehicleId,
                status: 'exit_complete'
            })
        });
        
        if (response.ok) {
            const vehicle = findVehicleById(vehicleId);
            showNotification(`차량 ${vehicle?.plate_number || vehicleId} 출차 완료`, 'success');
        }
    } catch (error) {
        console.error('Error completing vehicle exit:', error);
        showNotification('출차 완료 처리 실패', 'error');
    }
}

// 유틸리티 함수들
function findVehicleById(vehicleId) {
    const allVehicles = Object.values(vehicles).flat();
    return allVehicles.find(v => v.id === vehicleId);
}

function getTabName(tab) {
    const names = { entered: '입차', parking: '주차중', exiting: '출차' };
    return names[tab] || tab;
}

function getStatusText(status) {
    const texts = { 
        entered: '입차', 
        moving_to_parking: '주차이동',
        parked: '주차중', 
        exit_request: '출차요청',
        moving_to_exit: '출차이동'
    };
    return texts[status] || status;
}

function getTimeText(vehicle) {
    if (vehicle.entry_time) {
        return `입차: ${formatDateTime(vehicle.entry_time)}`;
    }
    if (vehicle.parked_time) {
        return `주차: ${formatDateTime(vehicle.parked_time)}`;
    }
    if (vehicle.exit_request_time) {
        return `출차요청: ${formatDateTime(vehicle.exit_request_time)}`;
    }
    if (vehicle.exit_time) {
        return `출차: ${formatDateTime(vehicle.exit_time)}`;
    }
    return '-';
}

function formatDateTime(isoString) {
    try {
        const date = new Date(isoString);
        return date.toLocaleString('ko-KR', {
            month: 'short',
            day: 'numeric',
            hour: '2-digit',
            minute: '2-digit'
        });
    } catch {
        return isoString;
    }
}

// 알림 표시
function showNotification(message, type = 'info') {
    // 기존 알림 제거
    const existingNotification = document.querySelector('.notification');
    if (existingNotification) {
        existingNotification.remove();
    }
    
    // 새 알림 생성
    const notification = document.createElement('div');
    notification.className = `notification notification-${type}`;
    notification.textContent = message;
    
    // 스타일 적용
    Object.assign(notification.style, {
        position: 'fixed',
        top: '20px',
        right: '20px',
        padding: '12px 20px',
        borderRadius: '8px',
        color: 'white',
        fontWeight: 'bold',
        zIndex: '9999',
        maxWidth: '400px',
        boxShadow: '0 4px 12px rgba(0,0,0,0.3)',
        animation: 'slideInRight 0.3s ease-out'
    });
    
    // 타입별 배경색
    const colors = {
        success: '#4CAF50',
        error: '#F44336',
        warning: '#FF9800',
        info: '#2196F3'
    };
    notification.style.backgroundColor = colors[type] || colors.info;
    
    document.body.appendChild(notification);
    
    // 3초 후 자동 제거
    setTimeout(() => {
        notification.style.animation = 'slideOutRight 0.3s ease-in';
        setTimeout(() => notification.remove(), 300);
    }, 3000);
}

// CSS 애니메이션 추가
const style = document.createElement('style');
style.textContent = `
    @keyframes slideInRight {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    
    @keyframes slideOutRight {
        from { transform: translateX(0); opacity: 1; }
        to { transform: translateX(100%); opacity: 0; }
    }
    
    .system-stats {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
        gap: 15px;
        margin-bottom: 20px;
    }
    
    .stat-card {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        padding: 15px;
        border-radius: 10px;
        text-align: center;
        box-shadow: 0 4px 8px rgba(0,0,0,0.1);
    }
    
    .stat-number {
        font-size: 24px;
        font-weight: bold;
        margin-bottom: 5px;
    }
    
    .stat-label {
        font-size: 12px;
        opacity: 0.9;
    }
    
    .system-info {
        background: #f8f9fa;
        padding: 20px;
        border-radius: 10px;
        border-left: 4px solid #007bff;
    }
    
    .info-grid {
        display: grid;
        gap: 10px;
    }
    
    .info-item {
        display: flex;
        justify-content: space-between;
        align-items: center;
        padding: 8px 0;
        border-bottom: 1px solid #e9ecef;
    }
    
    .info-item:last-child {
        border-bottom: none;
    }
    
    .status-active {
        color: #28a745;
        font-weight: bold;
    }
    
    .status-inactive {
        color: #dc3545;
        font-weight: bold;
    }
    
    .vehicle-actions {
        margin-top: 10px;
        display: flex;
        gap: 8px;
        flex-wrap: wrap;
    }
    
    .action-btn {
        padding: 6px 12px;
        border: none;
        border-radius: 6px;
        font-size: 12px;
        cursor: pointer;
        transition: all 0.2s;
        font-weight: 500;
    }
    
    .btn-primary {
        background: #007bff;
        color: white;
    }
    
    .btn-warning {
        background: #ffc107;
        color: #000;
    }
    
    .btn-success {
        background: #28a745;
        color: white;
    }
    
    .action-btn:hover {
        transform: translateY(-1px);
        box-shadow: 0 2px 4px rgba(0,0,0,0.2);
    }
    
    .info-row {
        margin: 5px 0;
    }
    
    .status-dot.connected {
        background-color: #28a745;
        box-shadow: 0 0 10px rgba(40, 167, 69, 0.5);
    }
    
    .status-dot.disconnected {
        background-color: #dc3545;
        box-shadow: 0 0 10px rgba(220, 53, 69, 0.5);
    }
    
    .empty-state {
        text-align: center;
        padding: 40px 20px;
        color: #666;
    }
    
    .vehicle-card {
        background: white;
        border-radius: 8px;
        padding: 15px;
        margin-bottom: 10px;
        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        border-left: 4px solid #007bff;
    }
    
    .vehicle-header {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 10px;
    }
    
    .vehicle-id {
        font-weight: bold;
        font-size: 16px;
    }
    
    .vehicle-status {
        padding: 4px 8px;
        border-radius: 4px;
        font-size: 12px;
        font-weight: bold;
    }
    
    .status-entered, .status-moving_to_parking {
        background: #e3f2fd;
        color: #1976d2;
    }
    
    .status-parked {
        background: #e8f5e8;
        color: #2e7d32;
    }
    
    .status-exit_request, .status-moving_to_exit {
        background: #fff3e0;
        color: #f57c00;
    }
`;
document.head.appendChild(style);

// 키보드 단축키
document.addEventListener('keydown', function(e) {
    if (e.ctrlKey || e.metaKey) {
        switch (e.key) {
            case '1':
                e.preventDefault();
                switchTab('entered');
                break;
            case '2':
                e.preventDefault();
                switchTab('parking');
                break;
            case '3':
                e.preventDefault();
                switchTab('exiting');
                break;
            case 'r':
                e.preventDefault();
                loadVehicleData();
                showNotification('데이터 새로고침', 'info');
                break;
        }
    }
});

// 페이지 언로드 시 WebSocket 정리
window.addEventListener('beforeunload', function() {
    if (websocket) {
        websocket.close();
    }
});