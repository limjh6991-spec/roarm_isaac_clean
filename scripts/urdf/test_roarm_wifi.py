#!/usr/bin/env python3
"""
RoArm-M3 WiFi 연결 테스트
간단한 명령으로 연결 확인 및 제어
"""

import requests
import time
import sys

# RoArm-M3 IP (AP 모드 기본값)
IP = "192.168.4.1"

def send_cmd(json_cmd, description=""):
    """JSON 명령 전송"""
    url = f"http://{IP}/js?json={json_cmd}"
    
    if description:
        print(f"\n{description}")
    
    try:
        response = requests.get(url, timeout=5)
        print(f"  ✅ 명령: {json_cmd}")
        print(f"  📨 응답: {response.text}")
        return response.text
    except requests.exceptions.ConnectionError:
        print(f"  ❌ 연결 실패!")
        print(f"     RoArm-M3 WiFi에 연결되었는지 확인하세요.")
        print(f"     WiFi: RoArm-M3")
        print(f"     비밀번호: 12345678")
        return None
    except requests.exceptions.Timeout:
        print(f"  ⏱️ 응답 시간 초과 (5초)")
        return None
    except Exception as e:
        print(f"  ❌ 오류: {e}")
        return None

def main():
    print("=" * 60)
    print("🤖 RoArm-M3 WiFi 연결 테스트")
    print("=" * 60)
    print(f"\n📡 연결 정보:")
    print(f"   WiFi SSID: RoArm-M3")
    print(f"   비밀번호: 12345678")
    print(f"   IP 주소: {IP}")
    print(f"   Web UI: http://{IP}")
    
    # 연결 테스트
    print(f"\n🔍 연결 테스트 중...")
    result = send_cmd('{"T":101}', "1️⃣ 초기화 (INIT)")
    
    if result is None:
        print("\n" + "=" * 60)
        print("❌ 연결 실패!")
        print("=" * 60)
        print("\n💡 문제 해결:")
        print("   1. RoArm-M3 전원 확인 (OLED 화면 켜져 있나요?)")
        print("   2. WiFi 연결 확인:")
        print("      nmcli connection show --active")
        print("   3. Ping 테스트:")
        print("      ping 192.168.4.1")
        print("   4. 브라우저 테스트:")
        print("      http://192.168.4.1")
        print("\n🛠️ 진단 스크립트 실행:")
        print("   bash ~/roarm_isaac_clean/scripts/diagnose_roarm_wifi.sh")
        sys.exit(1)
    
    print("\n✅ 연결 성공!")
    time.sleep(2)
    
    # 기본 동작 테스트
    print("\n" + "=" * 60)
    print("🎮 기본 동작 테스트")
    print("=" * 60)
    
    # 그리퍼 열기
    send_cmd('{"T":106,"cmd":1.57,"spd":0,"acc":0}', "2️⃣ 그리퍼 열기")
    time.sleep(1.5)
    
    # 그리퍼 닫기
    send_cmd('{"T":106,"cmd":3.14,"spd":0,"acc":0}', "3️⃣ 그리퍼 닫기")
    time.sleep(1.5)
    
    # Base 회전 (우측 45도)
    send_cmd('{"T":100,"joint":1,"cmd":0.785,"spd":50,"acc":30}', "4️⃣ Base 우측 45도 회전")
    time.sleep(2)
    
    # Base 중앙 복귀
    send_cmd('{"T":100,"joint":1,"cmd":0.0,"spd":50,"acc":30}', "5️⃣ Base 중앙 복귀")
    time.sleep(2)
    
    # 초기 위치
    send_cmd('{"T":101}', "6️⃣ 초기 위치로 복귀")
    time.sleep(1)
    
    print("\n" + "=" * 60)
    print("✅ 테스트 완료!")
    print("=" * 60)
    print("\n🎉 RoArm-M3 WiFi 연결이 정상 작동합니다!")
    print("\n📚 다음 단계:")
    print("   1. Web UI에서 수동 제어:")
    print("      http://192.168.4.1")
    print("   2. Python으로 자동 제어:")
    print("      이 스크립트를 수정하여 원하는 동작 추가")
    print("   3. JSON 명령어 참조:")
    print("      https://www.waveshare.com/wiki/RoArm-M3-S_JSON_Command_Meaning")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자에 의해 중단됨")
    except Exception as e:
        print(f"\n❌ 예상치 못한 오류: {e}")
        import traceback
        traceback.print_exc()
