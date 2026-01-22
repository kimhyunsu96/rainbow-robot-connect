#!/bin/bash
# 🚀 Rainbow Robot 웹 서버 빠른 시작 스크립트

echo ""
echo "================================"
echo "Rainbow Robot Web Control"
echo "================================"
echo ""

# 디렉토리 확인
WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[INFO] 웹 폴더: $WEB_DIR"

# 의존성 설치
echo ""
echo "[STEP 1] 필요한 패키지 설치 중..."
if [ -f "$WEB_DIR/requirements.txt" ]; then
    pip install -r "$WEB_DIR/requirements.txt" -q
    echo "✅ 패키지 설치 완료"
else
    echo "⚠️  requirements.txt를 찾을 수 없습니다"
fi

# 웹 서버 시작
echo ""
echo "[STEP 2] 웹 서버 시작 중..."
echo ""
echo "🌐 웹 서버가 시작되었습니다!"
echo "📱 브라우저에서 이 주소로 접속하세요:"
echo ""
echo "    🔗 http://localhost:5000"
echo ""
echo "또는 다른 기기에서:"
echo "    🔗 http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "종료하려면 Ctrl+C를 누르세요"
echo ""

# 웹 서버 실행
cd "$WEB_DIR"
python3 rb_web_test.py
