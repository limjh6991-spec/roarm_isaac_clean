#!/home/roarm_m3/isaacsim/python.sh
"""
USD 스키마 무결성 점검 스크립트
Isaac Sim 환경에서 pxr 모듈과 USD 스키마를 검증합니다.
"""

import sys
from pathlib import Path

# Isaac Sim 초기화 (pxr 모듈을 사용하기 위해 필수)
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

print("=" * 80)
print("🔍 USD/pxr 모듈 진단 도구")
print("=" * 80)
print()

# 1. Python 환경 정보
print("📍 Python 환경:")
print(f"   실행 파일: {sys.executable}")
print(f"   버전: {sys.version}")
print()

# 2. pxr 모듈 로딩 테스트
print("📦 pxr 모듈 로딩 테스트:")
try:
    from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
    
    # 모듈 경로
    pxr_module = sys.modules['pxr']
    print(f"✅ pxr 로딩 성공")
    print(f"   경로: {pxr_module.__file__}")
    print(f"   USD 버전: {Usd.GetVersion()}")
    print()
    
    # 3. USD 스키마 무결성 점검
    print("📋 USD 스키마 무결성 점검:")
    print()
    
    schema_categories = {
        "기본 지오메트리": {
            "UsdGeom.Xform": (UsdGeom, 'Xform'),
            "UsdGeom.Mesh": (UsdGeom, 'Mesh'),
            "UsdGeom.Cube": (UsdGeom, 'Cube'),
            "UsdGeom.Sphere": (UsdGeom, 'Sphere'),
            "UsdGeom.Cylinder": (UsdGeom, 'Cylinder'),
            "UsdGeom.Capsule": (UsdGeom, 'Capsule'),
        },
        "수학/변환": {
            "Gf.Vec3f": (Gf, 'Vec3f'),
            "Gf.Vec3d": (Gf, 'Vec3d'),
            "Gf.Matrix4d": (Gf, 'Matrix4d'),
            "Gf.Quatf": (Gf, 'Quatf'),
            "Gf.Transform": (Gf, 'Transform'),
        },
        "경로/레이어": {
            "Sdf.Path": (Sdf, 'Path'),
            "Sdf.Layer": (Sdf, 'Layer'),
        },
        "물리": {
            "UsdPhysics.RigidBodyAPI": (UsdPhysics, 'RigidBodyAPI'),
            "UsdPhysics.CollisionAPI": (UsdPhysics, 'CollisionAPI'),
            "PhysxSchema.PhysxArticulationAPI": (PhysxSchema, 'PhysxArticulationAPI'),
            "PhysxSchema.PhysxRigidBodyAPI": (PhysxSchema, 'PhysxRigidBodyAPI'),
        },
    }
    
    total_checks = 0
    passed_checks = 0
    failed_schemas = []
    
    for category, schemas in schema_categories.items():
        print(f"  📂 {category}:")
        for schema_name, (module, attr) in schemas.items():
            total_checks += 1
            exists = hasattr(module, attr)
            status = "✅" if exists else "❌"
            print(f"     {status} {schema_name}")
            if exists:
                passed_checks += 1
            else:
                failed_schemas.append(schema_name)
        print()
    
    # 4. 결과 요약
    print("=" * 80)
    print("📊 검사 결과 요약:")
    print(f"   총 점검: {total_checks}개")
    print(f"   통과: {passed_checks}개")
    print(f"   실패: {total_checks - passed_checks}개")
    
    if passed_checks == total_checks:
        print("\n🎉 모든 USD 스키마 정상!")
        sys.exit(0)
    else:
        print(f"\n⚠️ {len(failed_schemas)}개 스키마 누락:")
        for schema in failed_schemas:
            print(f"   - {schema}")
        simulation_app.close()
        sys.exit(1)
        
except ImportError as e:
    print(f"❌ pxr 모듈 로딩 실패!")
    print(f"   에러: {e}")
    print()
    print("🔍 sys.path 검사:")
    for i, path in enumerate(sys.path[:10], 1):
        print(f"   {i}. {path}")
    print()
    print("💡 해결 방법:")
    print("   1. Isaac Sim Python으로 실행: ~/isaacsim/python.sh this_script.py")
    print("   2. PYTHONPATH 설정 확인")
    simulation_app.close()
    sys.exit(2)

print("=" * 80)
simulation_app.close()
