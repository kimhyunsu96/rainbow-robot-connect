// JavaScript for Rainbow Robot Web Controller

const API_BASE = 'http://localhost:5000/api';

// 로그 함수
function log(message, type = 'info') {
    const logContainer = document.getElementById('log-container');
    const timestamp = new Date().toLocaleTimeString();
    const className = `log-${type}`;
    const logEntry = document.createElement('p');
    logEntry.className = className;
    logEntry.textContent = `[${timestamp}] ${message}`;
    logContainer.appendChild(logEntry);
    logContainer.scrollTop = logContainer.scrollHeight;
}

// 토스트 알림
function showToast(message, type = 'info') {
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    document.body.appendChild(toast);
    setTimeout(() => {
        toast.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

// API 호출 함수
async function apiCall(endpoint, method = 'GET', data = null) {
    try {
        const options = {
            method: method,
            headers: { 'Content-Type': 'application/json' },
        };
        
        if (data) {
            options.body = JSON.stringify(data);
        }
        
        const response = await fetch(`${API_BASE}${endpoint}`, options);
        
        if (!response.ok) {
            throw new Error(`API Error: ${response.status}`);
        }
        
        return await response.json();
    } catch (error) {
        log(`API 오류: ${error.message}`, 'error');
        throw error;
    }
}

// 상태 업데이트
async function updateStatus() {
    try {
        const status = await apiCall('/status');
        
        // 로봇 연결 상태 표시
        const robotConnected = status.robot_connected;
        const robotStatusElement = document.getElementById('robot-status');
        
        if (robotConnected) {
            robotStatusElement.textContent = '✅ 로봇 연결됨 (192.168.1.13)';
            robotStatusElement.style.color = 'var(--success-color)';
        } else {
            robotStatusElement.textContent = `⚠️ 로봇 미연결`;
            robotStatusElement.style.color = 'var(--danger-color)';
        }
        
        document.getElementById('busy-text').textContent = status.busy ? '예' : '아니오';
        document.getElementById('selected-file').textContent = 
            status.selected_file ? status.selected_file.split('/').pop() : '없음';
        
        // ServoJ 파라미터 표시 (제목에 현재 값 표시)
        const servojTitle = document.getElementById('servoj-title');
        if (servojTitle && status.servo_params) {
            const params = status.servo_params;
            servojTitle.textContent = `ServoJ 파라미터 (t1: ${params.t1}, t2: ${params.t2}, Gain: ${params.gain}, Alpha: ${params.alpha})`;
        }
        
    } catch (error) {
        log('상태 업데이트 실패', 'error');
    }
}

// 파일 목록 로드
async function loadMotionFiles() {
    try {
        log('모션 파일 목록 로드 중...');
        const result = await apiCall('/motions-list');
        
        const listContainer = document.getElementById('motions-list');
        listContainer.innerHTML = '';
        
        if (result.files && result.files.length > 0) {
            result.files.forEach(file => {
                const fileItem = document.createElement('div');
                fileItem.className = 'file-item';
                fileItem.innerHTML = `
                    <div>
                        <div class="file-name">📄 ${file.name}</div>
                        <div class="file-date">${file.relative} | ${new Date(file.modified).toLocaleString()}</div>
                    </div>
                    <button onclick="selectMotionFile('${file.path}', '${file.name}')" 
                            class="btn btn-primary" style="padding: 8px 12px; font-size: 0.9em;">선택</button>
                `;
                listContainer.appendChild(fileItem);
            });
            log(`${result.files.length}개의 모션 파일 로드됨`, 'success');
        } else {
            listContainer.innerHTML = '<p style="padding: 20px; text-align: center; color: #999;">모션 파일을 찾을 수 없습니다</p>';
            log('모션 파일이 없습니다', 'warning');
        }
    } catch (error) {
        log('파일 목록 로드 실패', 'error');
    }
}

// 모션 파일 선택
async function selectMotionFile(filepath, filename) {
    try {
        log(`파일 선택 중: ${filename}`);
        const result = await apiCall('/load-motion', 'POST', { filepath });
        
        if (result.success) {
            log(`파일 선택 완료: ${filename}`, 'success');
            document.getElementById('motion-selected-display').textContent = filename;
            
            // 선택된 파일 하이라이트
            document.querySelectorAll('.file-item').forEach(item => {
                item.classList.remove('selected');
            });
            event.target.closest('.file-item').classList.add('selected');
            
            updateStatus();
        } else {
            showToast('파일 선택 실패', 'error');
        }
    } catch (error) {
        showToast('파일 선택 오류', 'error');
    }
}

// Home 이동 실행
async function runHome() {
    try {
        // 로봇 연결 상태 확인
        const status = await apiCall('/status');
        if (!status.robot_connected) {
            showToast(`❌ 로봇이 연결되지 않았습니다: ${status.robot_error}`, 'error');
            return;
        }
        
        log('Home 이동 시작...');
        const result = await apiCall('/run-home', 'POST');
        
        if (result.success) {
            log('Home 이동 명령 전송됨', 'success');
            showToast('✅ Home 이동 시작', 'success');
        } else {
            showToast('❌ Home 이동 실패', 'error');
        }
        
        updateStatus();
    } catch (error) {
        showToast('❌ Home 이동 오류', 'error');
    }
}

// Home 위치 설정
async function setHomePose() {
    try {
        const input = document.getElementById('home-pose-input').value;
        const speed = parseFloat(document.getElementById('home-speed').value);
        const accel = parseFloat(document.getElementById('home-accel').value);
        
        if (!input) {
            showToast('Home 위치를 입력하세요', 'error');
            return;
        }
        
        // JSON 배열 파싱
        const pose = JSON.parse(input);
        if (!Array.isArray(pose) || pose.length !== 6) {
            showToast('6개의 조인트 값이 필요합니다', 'error');
            return;
        }
        
        log('Home 위치 설정 중...');
        const result = await apiCall('/home-pose', 'POST', {
            home_pose: pose,
            home_speed: speed,
            home_accel: accel,
        });
        
        if (result.success) {
            log(`Home 위치 설정 완료: ${JSON.stringify(pose)}`, 'success');
            showToast('Home 위치 설정 완료', 'success');
            updateStatus();
        }
    } catch (error) {
        showToast('Home 위치 설정 실패', 'error');
        log(error.message, 'error');
    }
}

// ServoJ 파라미터 업데이트
async function updateServoParams() {
    try {
        const t1 = parseFloat(document.getElementById('servo-t1').value);
        const t2 = parseFloat(document.getElementById('servo-t2').value);
        const gain = parseFloat(document.getElementById('servo-gain').value);
        const alpha = parseFloat(document.getElementById('servo-alpha').value);
        
        if (isNaN(t1) || isNaN(t2) || isNaN(gain) || isNaN(alpha)) {
            showToast('유효한 숫자를 입력하세요', 'error');
            return;
        }
        
        log('ServoJ 파라미터 업데이트 중...');
        const result = await apiCall('/servo-params', 'POST', {
            t1, t2, gain, alpha
        });
        
        if (result.success) {
            log('ServoJ 파라미터 업데이트 완료', 'success');
            showToast('ServoJ 파라미터 업데이트 완료', 'success');
            updateStatus();
        }
    } catch (error) {
        showToast('파라미터 업데이트 실패', 'error');
    }
}

// 모션 실행
async function runMotion() {
    try {
        // 로봇 연결 상태 확인
        const status = await apiCall('/status');
        if (!status.robot_connected) {
            showToast(`❌ 로봇이 연결되지 않았습니다: ${status.robot_error}`, 'error');
            return;
        }
        
        const selectedFile = document.getElementById('motion-selected-display').textContent;
        if (selectedFile === '없음') {
            showToast('먼저 모션 파일을 선택하세요', 'error');
            return;
        }
        
        log('모션 실행 시작...');
        const result = await apiCall('/run-motion', 'POST');
        
        if (result.success) {
            log('모션 실행 명령 전송됨', 'success');
            showToast('✅ 모션 실행 시작', 'success');
        } else {
            showToast('❌ 모션 실행 실패', 'error');
        }
        
        updateStatus();
    } catch (error) {
        showToast('❌ 모션 실행 오류', 'error');
    }
}

// 로봇 연결
async function connectRobot() {
    try {
        log('로봇 연결 시도 중...');
        const result = await apiCall('/connect-robot', 'POST');
        
        if (result.success) {
            log('로봇 연결 시도 성공', 'success');
            showToast('🔗 로봇 연결 시도 중...', 'info');
        } else {
            showToast('❌ 로봇 연결 실패', 'error');
        }
        
        updateStatus();
    } catch (error) {
        showToast('❌ 로봇 연결 오류', 'error');
    }
}

// MoveJ 이동
async function runMoveJ() {
    try {
        const jointsInput = document.getElementById('joints-input').value;
        const speed = parseFloat(document.getElementById('movej-speed').value);
        const accel = parseFloat(document.getElementById('movej-accel').value);
        
        log(`MoveJ 이동: ${jointsInput} (속도: ${speed}, 가속도: ${accel})`);
        
        const result = await apiCall('/run-movej', 'POST', {
            joints: jointsInput,
            speed: speed,
            accel: accel
        });
        
        if (result.success) {
            log('MoveJ 이동 명령 전송됨', 'success');
            showToast('✅ MoveJ 이동 시작', 'success');
        } else {
            showToast('❌ MoveJ 이동 실패', 'error');
        }
        
        updateStatus();
    } catch (error) {
        showToast('❌ MoveJ 이동 오류', 'error');
    }
}

// 페이지 초기화
async function initializePage() {
    log('Rainbow Robot 웹 제어기 시작...', 'success');
    
    // 초기 상태 로드
    await updateStatus();
    await loadMotionFiles();
    
    // 정기적 업데이트 (2초)
    setInterval(updateStatus, 2000);
    
    // 버튼 이벤트 바인딩
    document.getElementById('refresh-files-btn').addEventListener('click', loadMotionFiles);
    document.getElementById('connect-robot-btn').addEventListener('click', connectRobot);
    document.getElementById('run-home-btn').addEventListener('click', runHome);
    document.getElementById('run-movej-btn').addEventListener('click', runMoveJ);
    document.getElementById('update-servo-btn').addEventListener('click', updateServoParams);
    document.getElementById('run-motion-btn').addEventListener('click', runMotion);
    
    log('준비 완료! 모션 파일을 선택하여 시작하세요.', 'success');
}

// DOM 로드 후 초기화
document.addEventListener('DOMContentLoaded', initializePage);
