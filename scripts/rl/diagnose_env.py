#!/usr/bin/env python3
"""
RoArm M3 환경 진단 스크립트

7단계 진단 프로세스:
1. Observation Check
2. Coordinate Frame Alignment
3. Camera Validation (Vision 사용 시)
4. Physics & Collision
5. Reward Signal
6. Task Integration
7. Runtime Diagnostics

Usage:
    # 전체 진단
    python diagnose_env.py --full
    
    # 특정 항목 진단
    python diagnose_env.py --check observation
    python diagnose_env.py --check reward --verbose
"""

import argparse
import sys
import torch
import numpy as np
from pathlib import Path

# Isaac Sim 경로 추가
sys.path.append(str(Path.home() / "isaacsim"))

# 환경 import (실제 경로에 맞게 수정 필요)
try:
    from roarm_pick_place_env import RoArmPickPlaceEnv
except ImportError:
    print("⚠️ Warning: RoArmPickPlaceEnv를 import할 수 없습니다.")
    print("   이 스크립트는 실제 환경과 함께 실행해야 합니다.")
    RoArmPickPlaceEnv = None


class EnvironmentDiagnostics:
    """환경 진단 클래스"""
    
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.results = {}
        self.env = None
    
    def log(self, message, level="INFO"):
        """로그 출력"""
        symbols = {
            "INFO": "ℹ️",
            "SUCCESS": "✅",
            "WARNING": "⚠️",
            "ERROR": "❌",
            "DEBUG": "🔍"
        }
        print(f"{symbols.get(level, '')} {message}")
    
    def check_observation(self):
        """1. Observation Check"""
        self.log("=== 1. Observation Check ===", "INFO")
        
        try:
            # 환경 초기화 (헤드리스 모드)
            if RoArmPickPlaceEnv is None:
                self.log("환경을 import할 수 없어 모의 체크를 수행합니다.", "WARNING")
                # 모의 체크
                self.results["observation"] = {
                    "status": "SKIPPED",
                    "message": "환경 import 실패"
                }
                return False
            
            self.env = RoArmPickPlaceEnv(headless=True, num_envs=1)
            
            # Observation 가져오기
            obs = self.env.reset()
            
            self.log(f"Observation keys: {obs.keys()}", "DEBUG" if self.verbose else "INFO")
            
            # 정책 관측 확인
            if "policy" in obs:
                policy_obs = obs["policy"]
                self.log(f"Policy observation shape: {policy_obs.shape}", "INFO")
                self.log(f"Policy observation dtype: {policy_obs.dtype}", "DEBUG")
                
                # 예상 차원
                expected_dim = 25  # 현재 설정
                actual_dim = policy_obs.shape[-1]
                
                if actual_dim == expected_dim:
                    self.log(f"Observation dimension OK: {actual_dim}", "SUCCESS")
                else:
                    self.log(f"Observation dimension mismatch: {actual_dim} (expected: {expected_dim})", "WARNING")
            
            # 큐브 위치 확인
            if hasattr(self.env, 'cube'):
                cube_pos = self.env.cube.data.root_pos_w[0]
                self.log(f"Cube position: {cube_pos}", "INFO")
                
                # 값 범위 체크
                if torch.all((cube_pos >= 0.05) & (cube_pos <= 0.50)):
                    self.log("Cube position range OK", "SUCCESS")
                else:
                    self.log(f"Cube position out of expected range: {cube_pos}", "WARNING")
            else:
                self.log("Cube object not found in environment!", "ERROR")
                self.results["observation"] = {
                    "status": "FAILED",
                    "message": "Cube not registered"
                }
                return False
            
            self.results["observation"] = {
                "status": "PASSED",
                "obs_keys": list(obs.keys()),
                "policy_dim": policy_obs.shape[-1] if "policy" in obs else None,
                "cube_pos": cube_pos.tolist() if hasattr(self.env, 'cube') else None
            }
            return True
            
        except Exception as e:
            self.log(f"Observation check failed: {e}", "ERROR")
            self.results["observation"] = {
                "status": "ERROR",
                "message": str(e)
            }
            return False
    
    def check_coordinates(self):
        """2. Coordinate Frame Alignment"""
        self.log("\n=== 2. Coordinate Frame Alignment ===", "INFO")
        
        if self.env is None:
            self.log("환경이 초기화되지 않았습니다. Observation check를 먼저 실행하세요.", "WARNING")
            return False
        
        try:
            # EE 위치
            if hasattr(self.env, 'ee_frame'):
                ee_pos = self.env.ee_frame.data.target_pos_w[0, 0, :]
                self.log(f"EE position (World): {ee_pos}", "INFO")
            else:
                self.log("EE frame not found!", "ERROR")
                return False
            
            # 큐브 위치
            cube_pos = self.env.cube.data.root_pos_w[0]
            self.log(f"Cube position (World): {cube_pos}", "INFO")
            
            # 로봇 베이스 위치
            if hasattr(self.env, 'robot'):
                base_pos = self.env.robot.data.root_pos_w[0]
                self.log(f"Robot base (World): {base_pos}", "INFO")
            else:
                base_pos = torch.zeros(3)
            
            # 거리 계산
            ee_to_cube = torch.norm(ee_pos - cube_pos).item()
            self.log(f"EE to Cube distance: {ee_to_cube:.3f} m", "INFO")
            
            # 거리가 현실적인지 확인
            if 0.05 <= ee_to_cube <= 0.50:
                self.log("Distance is realistic", "SUCCESS")
            else:
                self.log(f"Distance seems unrealistic: {ee_to_cube:.3f} m", "WARNING")
            
            # 좌표계 일관성 확인
            if torch.allclose(base_pos, torch.zeros(3), atol=0.01):
                self.log("Robot base at origin - using World frame", "INFO")
            else:
                self.log(f"Robot base offset: {base_pos}", "INFO")
                self.log("Recommend using relative coordinates", "WARNING")
            
            self.results["coordinates"] = {
                "status": "PASSED",
                "ee_pos": ee_pos.tolist(),
                "cube_pos": cube_pos.tolist(),
                "base_pos": base_pos.tolist(),
                "ee_to_cube_dist": ee_to_cube
            }
            return True
            
        except Exception as e:
            self.log(f"Coordinate check failed: {e}", "ERROR")
            self.results["coordinates"] = {
                "status": "ERROR",
                "message": str(e)
            }
            return False
    
    def check_physics(self):
        """4. Physics & Collision"""
        self.log("\n=== 4. Physics & Collision ===", "INFO")
        
        if self.env is None:
            self.log("환경이 초기화되지 않았습니다.", "WARNING")
            return False
        
        try:
            # 큐브 속성 확인
            cube = self.env.cube
            
            # Dynamic vs Kinematic
            # (Isaac Lab API에 따라 구현 필요)
            self.log("Checking cube physics properties...", "INFO")
            
            # 큐브 속도 확인 (움직이는지)
            cube_vel = cube.data.root_lin_vel_w[0]
            vel_norm = torch.norm(cube_vel).item()
            self.log(f"Cube velocity: {vel_norm:.3f} m/s", "INFO")
            
            if vel_norm < 1e-3:
                self.log("Cube is static (good for initial state)", "SUCCESS")
            else:
                self.log(f"Cube is moving: {vel_norm:.3f} m/s", "INFO")
            
            # URDF 버전 확인
            urdf_path = Path.home() / "roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_multiprim.urdf"
            if urdf_path.exists():
                with open(urdf_path, 'r') as f:
                    urdf_content = f.read()
                    
                    # 그리퍼 타입 확인
                    if 'type="prismatic"' in urdf_content and 'gripper' in urdf_content.lower():
                        self.log("✅ URDF: Gripper is Prismatic", "SUCCESS")
                    else:
                        self.log("⚠️ URDF: Gripper type unclear", "WARNING")
                    
                    # 마찰 계수 확인
                    if 'mu1="0.6"' in urdf_content or 'mu2="0.6"' in urdf_content:
                        self.log("✅ URDF: High friction (0.6) detected", "SUCCESS")
                    else:
                        self.log("⚠️ URDF: Low friction or not set", "WARNING")
            else:
                self.log(f"URDF file not found: {urdf_path}", "WARNING")
            
            self.results["physics"] = {
                "status": "PASSED",
                "cube_velocity": vel_norm,
                "urdf_path": str(urdf_path),
                "urdf_exists": urdf_path.exists()
            }
            return True
            
        except Exception as e:
            self.log(f"Physics check failed: {e}", "ERROR")
            self.results["physics"] = {
                "status": "ERROR",
                "message": str(e)
            }
            return False
    
    def check_reward(self):
        """5. Reward Signal"""
        self.log("\n=== 5. Reward Signal ===", "INFO")
        
        if self.env is None:
            self.log("환경이 초기화되지 않았습니다.", "WARNING")
            return False
        
        try:
            # 여러 스텝 실행하며 보상 확인
            self.env.reset()
            
            rewards = []
            for i in range(10):
                # 무작위 액션
                action = torch.randn(1, self.env.action_space.shape[0]) * 0.1
                obs, reward, done, info = self.env.step(action)
                rewards.append(reward.item())
                
                if self.verbose:
                    self.log(f"Step {i}: reward={reward.item():.3f}", "DEBUG")
            
            # 보상 통계
            rewards = np.array(rewards)
            self.log(f"Reward mean: {rewards.mean():.3f}", "INFO")
            self.log(f"Reward std: {rewards.std():.3f}", "INFO")
            self.log(f"Reward range: [{rewards.min():.3f}, {rewards.max():.3f}]", "INFO")
            
            # Time penalty만 받고 있는지 확인
            if np.allclose(rewards, -0.01, atol=0.001):
                self.log("⚠️ Only time penalty detected! No task rewards.", "WARNING")
                self.log("   This suggests the robot is not interacting with the cube.", "WARNING")
            else:
                self.log("✅ Task rewards detected", "SUCCESS")
            
            # grasp_valid 조건 확인 (코드 분석)
            env_file = Path(__file__).parent / "roarm_pick_place_env.py"
            if env_file.exists():
                with open(env_file, 'r') as f:
                    content = f.read()
                    
                    if 'ee_to_cube_dist < 0.08' in content:
                        self.log("⚠️ GRIP condition: ee_to_cube_dist < 0.08 (strict)", "WARNING")
                        self.log("   Recommend: < 0.10", "INFO")
                    
                    if 'gripper_width < 0.02' in content:
                        self.log("⚠️ GRIP condition: gripper_width < 0.02 (strict)", "WARNING")
                        self.log("   Recommend: < 0.03", "INFO")
            
            self.results["reward"] = {
                "status": "PASSED",
                "mean_reward": float(rewards.mean()),
                "std_reward": float(rewards.std()),
                "min_reward": float(rewards.min()),
                "max_reward": float(rewards.max())
            }
            return True
            
        except Exception as e:
            self.log(f"Reward check failed: {e}", "ERROR")
            self.results["reward"] = {
                "status": "ERROR",
                "message": str(e)
            }
            return False
    
    def check_integration(self):
        """6. Task Integration"""
        self.log("\n=== 6. Task Integration ===", "INFO")
        
        if self.env is None:
            self.log("환경이 초기화되지 않았습니다.", "WARNING")
            return False
        
        try:
            # Scene objects 확인
            if hasattr(self.env, 'scene'):
                self.log(f"Scene articulations: {list(self.env.scene.articulations.keys())}", "INFO")
                self.log(f"Scene rigid_objects: {list(self.env.scene.rigid_objects.keys())}", "INFO")
                
                # 필수 오브젝트 확인
                required = ["robot", "cube"]
                for obj_name in required:
                    if obj_name in self.env.scene.articulations or obj_name in self.env.scene.rigid_objects:
                        self.log(f"✅ '{obj_name}' registered", "SUCCESS")
                    else:
                        self.log(f"❌ '{obj_name}' NOT registered!", "ERROR")
                        self.results["integration"] = {
                            "status": "FAILED",
                            "message": f"{obj_name} not in scene"
                        }
                        return False
            else:
                self.log("Scene attribute not found!", "ERROR")
                return False
            
            # Episode 길이 확인
            if hasattr(self.env, 'max_episode_length'):
                max_ep_len = self.env.max_episode_length
                self.log(f"Max episode length: {max_ep_len}", "INFO")
                
                if max_ep_len == 200:
                    self.log("✅ Episode length is standard (200)", "SUCCESS")
                else:
                    self.log(f"Episode length: {max_ep_len} (non-standard)", "INFO")
            
            self.results["integration"] = {
                "status": "PASSED",
                "scene_objects": {
                    "articulations": list(self.env.scene.articulations.keys()) if hasattr(self.env, 'scene') else [],
                    "rigid_objects": list(self.env.scene.rigid_objects.keys()) if hasattr(self.env, 'scene') else []
                }
            }
            return True
            
        except Exception as e:
            self.log(f"Integration check failed: {e}", "ERROR")
            self.results["integration"] = {
                "status": "ERROR",
                "message": str(e)
            }
            return False
    
    def run_runtime_diagnostics(self):
        """7. Runtime Diagnostics"""
        self.log("\n=== 7. Runtime Diagnostics ===", "INFO")
        
        if self.env is None:
            self.log("환경이 초기화되지 않았습니다.", "WARNING")
            return False
        
        try:
            self.env.reset()
            
            # 100 스텝 실행
            self.log("Running 100 steps...", "INFO")
            
            diagnostics = {
                "ee_to_cube_dists": [],
                "cube_heights": [],
                "gripper_widths": [],
                "rewards": []
            }
            
            for i in range(100):
                action = torch.randn(1, self.env.action_space.shape[0]) * 0.1
                obs, reward, done, info = self.env.step(action)
                
                # 데이터 수집
                ee_pos = self.env.ee_frame.data.target_pos_w[0, 0, :]
                cube_pos = self.env.cube.data.root_pos_w[0]
                
                ee_to_cube = torch.norm(ee_pos - cube_pos).item()
                cube_height = cube_pos[2].item()
                
                # 그리퍼 너비 (prismatic joints)
                if hasattr(self.env.robot.data, 'joint_pos'):
                    joint_pos = self.env.robot.data.joint_pos[0]
                    if len(joint_pos) >= 8:
                        gripper_width = (joint_pos[6] + joint_pos[7]).item()
                    else:
                        gripper_width = 0.0
                else:
                    gripper_width = 0.0
                
                diagnostics["ee_to_cube_dists"].append(ee_to_cube)
                diagnostics["cube_heights"].append(cube_height)
                diagnostics["gripper_widths"].append(gripper_width)
                diagnostics["rewards"].append(reward.item())
                
                if done:
                    self.log(f"Episode done at step {i}", "INFO")
                    self.env.reset()
            
            # 통계
            for key, values in diagnostics.items():
                values = np.array(values)
                self.log(f"{key}: mean={values.mean():.3f}, std={values.std():.3f}", "INFO")
            
            # 분석
            ee_dists = np.array(diagnostics["ee_to_cube_dists"])
            if ee_dists.min() < 0.10:
                self.log("✅ EE approached cube (< 10cm)", "SUCCESS")
            else:
                self.log(f"⚠️ EE never close to cube (min dist: {ee_dists.min():.3f})", "WARNING")
            
            cube_heights = np.array(diagnostics["cube_heights"])
            if cube_heights.max() > 0.05:
                self.log("✅ Cube lifted above 5cm", "SUCCESS")
            else:
                self.log(f"⚠️ Cube never lifted (max height: {cube_heights.max():.3f})", "WARNING")
            
            self.results["runtime"] = {
                "status": "PASSED",
                "diagnostics": {k: {
                    "mean": float(np.mean(v)),
                    "std": float(np.std(v)),
                    "min": float(np.min(v)),
                    "max": float(np.max(v))
                } for k, v in diagnostics.items()}
            }
            return True
            
        except Exception as e:
            self.log(f"Runtime diagnostics failed: {e}", "ERROR")
            self.results["runtime"] = {
                "status": "ERROR",
                "message": str(e)
            }
            return False
    
    def generate_report(self):
        """진단 리포트 생성"""
        self.log("\n" + "="*60, "INFO")
        self.log("=== DIAGNOSTIC REPORT ===", "INFO")
        self.log("="*60, "INFO")
        
        total_checks = len(self.results)
        passed = sum(1 for r in self.results.values() if r.get("status") == "PASSED")
        failed = sum(1 for r in self.results.values() if r.get("status") == "FAILED")
        errors = sum(1 for r in self.results.values() if r.get("status") == "ERROR")
        
        self.log(f"\nTotal checks: {total_checks}", "INFO")
        self.log(f"Passed: {passed} ✅", "SUCCESS")
        self.log(f"Failed: {failed} ❌", "ERROR" if failed > 0 else "INFO")
        self.log(f"Errors: {errors} 💥", "ERROR" if errors > 0 else "INFO")
        
        self.log("\nDetailed results:", "INFO")
        for check_name, result in self.results.items():
            status = result.get("status", "UNKNOWN")
            symbol = {"PASSED": "✅", "FAILED": "❌", "ERROR": "💥", "SKIPPED": "⏭️"}.get(status, "❓")
            self.log(f"  {symbol} {check_name}: {status}", "INFO")
            
            if "message" in result:
                self.log(f"     → {result['message']}", "INFO")
        
        # 권장 사항
        self.log("\n" + "="*60, "INFO")
        self.log("=== RECOMMENDATIONS ===", "INFO")
        self.log("="*60, "INFO")
        
        if failed > 0 or errors > 0:
            self.log("⚠️ Fix the issues above before training!", "WARNING")
            self.log("   Refer to docs/rl/DIAGNOSTIC_CHECKLIST.md for solutions.", "INFO")
        else:
            self.log("✅ All checks passed! Ready to train.", "SUCCESS")
            self.log("   Run: PYTHONUNBUFFERED=1 ~/isaacsim/python.sh train_dense_reward.py --timesteps 50000", "INFO")
        
        return passed == total_checks
    
    def cleanup(self):
        """환경 정리"""
        if self.env is not None:
            try:
                self.env.close()
            except:
                pass


def main():
    parser = argparse.ArgumentParser(description="RoArm M3 환경 진단")
    parser.add_argument("--check", type=str, choices=[
        "observation", "coordinates", "physics", "reward", "integration", "runtime"
    ], help="특정 항목만 체크")
    parser.add_argument("--full", action="store_true", help="전체 진단 수행")
    parser.add_argument("--verbose", action="store_true", help="상세 로그 출력")
    
    args = parser.parse_args()
    
    diag = EnvironmentDiagnostics(verbose=args.verbose)
    
    try:
        if args.check == "observation" or args.full:
            diag.check_observation()
        
        if args.check == "coordinates" or args.full:
            diag.check_coordinates()
        
        if args.check == "physics" or args.full:
            diag.check_physics()
        
        if args.check == "reward" or args.full:
            diag.check_reward()
        
        if args.check == "integration" or args.full:
            diag.check_integration()
        
        if args.check == "runtime" or args.full:
            diag.run_runtime_diagnostics()
        
        # 리포트 생성
        success = diag.generate_report()
        
        sys.exit(0 if success else 1)
        
    finally:
        diag.cleanup()


if __name__ == "__main__":
    main()
