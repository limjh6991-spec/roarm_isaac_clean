#!/bin/bash
# RoArm-M3 WiFi 연결 진단 스크립트

echo "🔍 RoArm-M3 WiFi 연결 진단"
echo "================================"
echo ""

# 1. WiFi 어댑터 확인
echo "1️⃣ WiFi 어댑터 확인..."
if nmcli device status | grep -q "wifi"; then
    echo "   ✅ WiFi 어댑터 발견"
    nmcli device status | grep wifi
else
    echo "   ❌ WiFi 어댑터를 찾을 수 없습니다"
    exit 1
fi
echo ""

# 2. RoArm-M3 WiFi 스캔
echo "2️⃣ RoArm-M3 WiFi 검색 중..."
echo "   (30초 대기...)"
sleep 2

nmcli device wifi rescan
sleep 3

if nmcli device wifi list | grep -q "RoArm-M3"; then
    echo "   ✅ RoArm-M3 WiFi 발견!"
    nmcli device wifi list | grep "RoArm-M3"
    echo ""
    echo "   📶 신호 강도:"
    nmcli device wifi list | grep "RoArm-M3" | awk '{print "      Signal: " $8}'
else
    echo "   ❌ RoArm-M3 WiFi를 찾을 수 없습니다"
    echo ""
    echo "   💡 문제 해결:"
    echo "      1. RoArm-M3 전원이 켜져 있는지 확인"
    echo "      2. OLED 화면에 'AP:RoArm-M3' 표시되는지 확인"
    echo "      3. RoArm-M3를 더 가까이 가져가기"
    echo "      4. 전원을 껐다 켜보기"
    echo ""
    echo "   📋 주변 WiFi 목록:"
    nmcli device wifi list | head -10
    exit 1
fi
echo ""

# 3. 연결 시도
echo "3️⃣ RoArm-M3에 연결 시도..."
echo "   비밀번호: 12345678"

# 기존 연결 삭제 (있을 경우)
nmcli connection delete "RoArm-M3" 2>/dev/null

# 새로 연결
if nmcli device wifi connect "RoArm-M3" password "12345678"; then
    echo "   ✅ WiFi 연결 성공!"
else
    echo "   ❌ 연결 실패"
    echo ""
    echo "   💡 비밀번호 확인:"
    echo "      - 정확히: 12345678 (숫자 8개)"
    echo "      - 대소문자 없음"
    exit 1
fi
echo ""

# 4. IP 확인
echo "4️⃣ IP 주소 확인..."
echo "   ⏳ DHCP IP 할당 대기 중 (10초)..."
sleep 10

# wlp7s0 인터페이스 사용 (시스템에 맞게)
WIFI_INTERFACE=$(nmcli device status | grep "wifi.*연결됨" | awk '{print $1}' | head -1)
echo "   WiFi 인터페이스: $WIFI_INTERFACE"

IP=$(ip addr show $WIFI_INTERFACE 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)
if [ -n "$IP" ]; then
    echo "   ✅ IP 할당 받음: $IP"
else
    echo "   ❌ IP를 받지 못했습니다"
    exit 1
fi
echo ""

# 5. Ping 테스트
echo "5️⃣ RoArm-M3 연결 테스트 (Ping)..."
if ping -c 3 -W 2 192.168.4.1 > /dev/null 2>&1; then
    echo "   ✅ Ping 성공! RoArm-M3와 통신 가능"
    ping -c 3 192.168.4.1 | tail -2
else
    echo "   ❌ Ping 실패"
    echo ""
    echo "   💡 문제:"
    echo "      - WiFi는 연결되었지만 RoArm-M3와 통신 불가"
    echo "      - 방화벽 확인: sudo ufw status"
    exit 1
fi
echo ""

# 6. HTTP 테스트
echo "6️⃣ Web 접속 테스트..."
if command -v curl &> /dev/null; then
    if curl -s --connect-timeout 5 http://192.168.4.1 > /dev/null; then
        echo "   ✅ Web 서버 응답 정상!"
    else
        echo "   ⚠️ Web 서버 응답 없음"
        echo "      (브라우저에서 직접 확인 필요)"
    fi
else
    echo "   ⚠️ curl 없음 (브라우저에서 확인)"
fi
echo ""

# 7. Python 테스트
echo "7️⃣ Python 연결 테스트..."
if command -v python3 &> /dev/null; then
    python3 << 'EOF'
import sys
try:
    import requests
    response = requests.get("http://192.168.4.1/js?json={\"T\":101}", timeout=3)
    if response.status_code == 200:
        print("   ✅ Python HTTP 요청 성공!")
        print(f"      응답: {response.text[:50]}...")
    else:
        print(f"   ⚠️ 응답 코드: {response.status_code}")
except ImportError:
    print("   ⚠️ requests 모듈 없음")
    print("      설치: pip install requests")
    sys.exit(0)
except Exception as e:
    print(f"   ❌ 오류: {e}")
    sys.exit(1)
EOF
else
    echo "   ⚠️ Python3 없음"
fi
echo ""

# 결과
echo "================================"
echo "✅ 진단 완료!"
echo ""
echo "📝 연결 정보:"
echo "   WiFi: RoArm-M3"
echo "   IP: $IP"
echo "   Gateway: 192.168.4.1"
echo ""
echo "🌐 브라우저에서 접속:"
echo "   http://192.168.4.1"
echo ""
echo "🐍 Python 테스트:"
echo "   cd ~/roarm_isaac_clean"
echo "   python3 test_roarm_wifi.py"
echo ""

# 브라우저 자동 열기
read -p "브라우저를 자동으로 열까요? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    xdg-open http://192.168.4.1 2>/dev/null || echo "브라우저를 수동으로 여세요: http://192.168.4.1"
fi
