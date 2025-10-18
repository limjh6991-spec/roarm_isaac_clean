#!/usr/bin/env python3
"""
온톨로지 문제 등록 도구

신규 문제를 온톨로지에 자동으로 등록하는 CLI 도구입니다.
템플릿 기반으로 .ttl 파일을 생성하고 자동으로 ID를 할당합니다.

사용법:
    # 기본 사용
    python add_problem.py \\
        --name "새로운 문제" \\
        --type EnvironmentProblem \\
        --severity CRITICAL \\
        --symptom "ModuleNotFoundError: some_module" \\
        --description "상세 설명..."
    
    # 솔루션과 함께 등록
    python add_problem.py \\
        --name "충돌 문제" \\
        --type PhysicsProblem \\
        --severity HIGH \\
        --symptom "Collision detection failed" \\
        --solution "충돌 메시 단순화" \\
        --solution-type ConfigurationSolution
"""

import argparse
import sys
from pathlib import Path
from datetime import datetime
from typing import List, Optional
import re

try:
    from rdflib import Graph, Namespace
    from rdflib.namespace import RDF
except ImportError:
    print("❌ rdflib 라이브러리가 필요합니다")
    print("설치 명령: pip install rdflib")
    sys.exit(1)


# 네임스페이스
ROARM = Namespace("http://roarm.ai/ontology#")

# 문제 유형
PROBLEM_TYPES = [
    "EnvironmentProblem",
    "PhysicsProblem",
    "APIProblem",
    "ConfigurationProblem",
    "DependencyProblem",
]

# 심각도
SEVERITIES = ["CRITICAL", "HIGH", "MEDIUM", "LOW"]

# 상태
STATUSES = ["OPEN", "IN_PROGRESS", "SOLVED", "RECURRING"]

# 솔루션 유형
SOLUTION_TYPES = [
    "ScriptSolution",
    "ConfigurationSolution",
    "DocumentationSolution",
    "WorkaroundSolution",
]


class ProblemRegistry:
    """문제 등록 관리 클래스"""
    
    def __init__(self, ontology_dir: Path):
        self.ontology_dir = ontology_dir
        self.instances_dir = ontology_dir / "instances"
        self.instances_dir.mkdir(parents=True, exist_ok=True)
        
    def generate_problem_id(self, name: str) -> str:
        """문제 이름에서 ID 생성"""
        # 특수문자 제거 및 공백을 밑줄로
        problem_id = re.sub(r'[^a-zA-Z0-9가-힣\s]', '', name)
        problem_id = problem_id.strip().replace(' ', '_').lower()
        
        # 한글을 영문으로 간단 변환 (선택적)
        # 여기서는 그대로 유지
        
        # 중복 확인
        base_id = problem_id
        counter = 1
        while (self.instances_dir / f"{problem_id}_problem.ttl").exists():
            problem_id = f"{base_id}_{counter}"
            counter += 1
        
        return problem_id
    
    def check_duplicates(self, symptom: str) -> List[str]:
        """증상으로 중복 문제 검색"""
        duplicates = []
        
        # 모든 .ttl 파일 로드
        g = Graph()
        for ttl_file in self.instances_dir.glob("*_problem.ttl"):
            try:
                g.parse(ttl_file, format="turtle")
            except:
                continue
        
        # symptom 검색
        from rdflib import Literal
        for s, p, o in g:
            if isinstance(o, Literal) and symptom.lower() in str(o).lower():
                problem_id = str(s).split('#')[-1]
                if problem_id not in duplicates:
                    duplicates.append(problem_id)
        
        return duplicates
    
    def create_problem_ttl(
        self,
        problem_id: str,
        name: str,
        problem_type: str,
        severity: str,
        symptom: str,
        description: str,
        status: str = "OPEN",
        solution_name: Optional[str] = None,
        solution_type: Optional[str] = None,
    ) -> str:
        """문제 .ttl 파일 내용 생성"""
        timestamp = datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ")
        date = datetime.now().strftime("%Y-%m-%d")
        
        # 솔루션 섹션
        solution_section = ""
        if solution_name and solution_type:
            solution_id = f"{problem_id}_solution"
            solution_section = f"""
# ----------------------------------------------------------------
# 해결책
# ----------------------------------------------------------------
:{problem_id}_problem :hasSolution :{solution_id} .

:{solution_id} a :{solution_type} ;
    rdfs:label "{solution_name}" ;
    rdfs:comment "자동 생성된 솔루션 - 추가 정보 필요" ;
    :successRate 0.0 ;  # 미검증
    :applicationScope "LOCAL" ;
    :status "PROPOSED" .
"""
        
        ttl_content = f"""@prefix : <http://roarm.ai/ontology#> .
@prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> .
@prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> .
@prefix xsd: <http://www.w3.org/2001/XMLSchema#> .

# ================================================================
# 문제: {name}
# ================================================================
# 생성일: {date}
# 타입: {problem_type}
# 심각도: {severity}
# 상태: {status}
# ================================================================

# ----------------------------------------------------------------
# 문제 정의
# ----------------------------------------------------------------
:{problem_id}_problem a :{problem_type} , :Problem ;
    rdfs:label "{name}" ;
    rdfs:comment "{description}" ;
    :severity "{severity}" ;
    :status "{status}" ;
    :symptom "{symptom}" ;
    :firstOccurrence "{timestamp}"^^xsd:dateTime ;
    :occurrenceCount 1 .
{solution_section}
# ----------------------------------------------------------------
# 추가 정보
# ----------------------------------------------------------------
# TODO: 문서, 스크립트, 검증 방법 등 추가
#
# 예시:
# :{problem_id}_problem :documents :some_guide_doc .
# :{problem_id}_problem :relatedTo :some_component .
# :{problem_id}_problem :causedBy :root_cause .
#
# :some_guide_doc a :Document ;
#     rdfs:label "해결 가이드" ;
#     :path "docs/guides/solution.md" .
# ----------------------------------------------------------------

# ================================================================
# End of {problem_id}_problem.ttl
# ================================================================
"""
        return ttl_content
    
    def register_problem(
        self,
        name: str,
        problem_type: str,
        severity: str,
        symptom: str,
        description: str,
        status: str = "OPEN",
        solution_name: Optional[str] = None,
        solution_type: Optional[str] = None,
        check_duplicates: bool = True,
    ) -> Path:
        """문제 등록"""
        # 1. 중복 확인
        if check_duplicates:
            duplicates = self.check_duplicates(symptom)
            if duplicates:
                print(f"⚠️  유사한 증상을 가진 문제가 {len(duplicates)}개 발견되었습니다:")
                for dup in duplicates[:3]:
                    print(f"   • {dup}")
                
                response = input("\n계속 진행하시겠습니까? (y/N): ")
                if response.lower() != 'y':
                    print("❌ 등록 취소")
                    sys.exit(0)
        
        # 2. ID 생성
        problem_id = self.generate_problem_id(name)
        print(f"\n생성된 문제 ID: {problem_id}_problem")
        
        # 3. .ttl 파일 생성
        ttl_content = self.create_problem_ttl(
            problem_id, name, problem_type, severity,
            symptom, description, status,
            solution_name, solution_type
        )
        
        # 4. 파일 저장
        output_file = self.instances_dir / f"{problem_id}_problem.ttl"
        output_file.write_text(ttl_content, encoding='utf-8')
        
        # 5. 검증
        g = Graph()
        try:
            g.parse(output_file, format="turtle")
            print(f"✅ 검증 성공: {len(g)} 트리플")
        except Exception as e:
            print(f"❌ 검증 실패: {e}")
            output_file.unlink()  # 실패 시 파일 삭제
            raise
        
        return output_file


def main():
    parser = argparse.ArgumentParser(
        description="온톨로지 문제 등록 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예제:
  # 환경 문제 등록
  python add_problem.py \\
    --name "Python 패키지 누락" \\
    --type EnvironmentProblem \\
    --severity HIGH \\
    --symptom "ModuleNotFoundError: numpy" \\
    --description "numpy 패키지가 설치되지 않음"
  
  # 솔루션과 함께 등록
  python add_problem.py \\
    --name "충돌 메시 문제" \\
    --type PhysicsProblem \\
    --severity MEDIUM \\
    --symptom "Collision mesh too complex" \\
    --description "충돌 메시가 너무 복잡하여 성능 저하" \\
    --solution "메시 단순화 스크립트 사용" \\
    --solution-type ScriptSolution
        """
    )
    
    parser.add_argument(
        "--name",
        required=True,
        help="문제 이름 (예: 'pxr 모듈 환경 설정 문제')"
    )
    parser.add_argument(
        "--type",
        choices=PROBLEM_TYPES,
        required=True,
        help=f"문제 유형: {', '.join(PROBLEM_TYPES)}"
    )
    parser.add_argument(
        "--severity",
        choices=SEVERITIES,
        required=True,
        help=f"심각도: {', '.join(SEVERITIES)}"
    )
    parser.add_argument(
        "--symptom",
        required=True,
        help="증상 설명 (예: 'ModuleNotFoundError: pxr')"
    )
    parser.add_argument(
        "--description",
        required=True,
        help="상세 설명"
    )
    parser.add_argument(
        "--status",
        choices=STATUSES,
        default="OPEN",
        help=f"상태 (기본: OPEN): {', '.join(STATUSES)}"
    )
    parser.add_argument(
        "--solution",
        help="솔루션 이름 (선택)"
    )
    parser.add_argument(
        "--solution-type",
        choices=SOLUTION_TYPES,
        help=f"솔루션 유형: {', '.join(SOLUTION_TYPES)}"
    )
    parser.add_argument(
        "--no-check-duplicates",
        action="store_true",
        help="중복 확인 생략"
    )
    parser.add_argument(
        "--ontology-dir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "ontology",
        help="ontology/ 디렉토리 경로"
    )
    
    args = parser.parse_args()
    
    # 솔루션 인자 검증
    if args.solution and not args.solution_type:
        parser.error("--solution 사용 시 --solution-type 필수")
    if args.solution_type and not args.solution:
        parser.error("--solution-type 사용 시 --solution 필수")
    
    # 온톨로지 디렉토리 확인
    if not args.ontology_dir.exists():
        print(f"❌ 온톨로지 디렉토리를 찾을 수 없습니다: {args.ontology_dir}")
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("🔧 온톨로지 문제 등록 도구")
    print("=" * 60)
    print()
    
    # 문제 등록
    registry = ProblemRegistry(args.ontology_dir)
    
    try:
        output_file = registry.register_problem(
            name=args.name,
            problem_type=args.type,
            severity=args.severity,
            symptom=args.symptom,
            description=args.description,
            status=args.status,
            solution_name=args.solution,
            solution_type=args.solution_type,
            check_duplicates=not args.no_check_duplicates,
        )
        
        print(f"\n✅ 문제 등록 완료!")
        print(f"📄 파일: {output_file}")
        print()
        print("다음 단계:")
        print("  1. 생성된 .ttl 파일을 열어 추가 정보 입력")
        print("  2. 문서/스크립트 연결")
        print("  3. 온톨로지 질의로 확인:")
        print(f"     python scripts/ontology/query_ontology.py --query project_status")
        print()
        
    except Exception as e:
        print(f"\n❌ 등록 실패: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
