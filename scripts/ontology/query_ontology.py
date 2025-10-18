#!/usr/bin/env python3
"""
SPARQL 질의 실행 도구

온톨로지 저장소에 SPARQL 질의를 실행하여 문제 진단, 솔루션 검색 등을 수행합니다.

사용법:
    python scripts/ontology/query_ontology.py --query critical_open_problems
    python scripts/ontology/query_ontology.py --query all_about_pxr
    python scripts/ontology/query_ontology.py --custom-query queries/my_query.sparql
"""

import argparse
import sys
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Any

try:
    from rdflib import Graph, Namespace, RDF, RDFS
    from rdflib.plugins.sparql import prepareQuery
except ImportError:
    print("❌ rdflib가 설치되지 않았습니다.")
    print("설치: pip install rdflib")
    sys.exit(1)

# 프로젝트 루트 디렉토리
PROJECT_ROOT = Path(__file__).parent.parent.parent
ONTOLOGY_DIR = PROJECT_ROOT / "ontology"
QUERIES_DIR = ONTOLOGY_DIR / "queries"

# 네임스페이스 정의
ROARM = Namespace("http://roarm.ai/ontology#")


class OntologyQuery:
    """온톨로지 SPARQL 질의 실행"""
    
    def __init__(self):
        self.graph = Graph()
        self.graph.bind("", ROARM)
        self.graph.bind("rdfs", RDFS)
        
    def load_ontology(self):
        """온톨로지 파일 로드"""
        print("📚 온톨로지 로딩 중...")
        
        # 코어 온톨로지
        core_files = [
            ONTOLOGY_DIR / "roarm_domain.ttl",
        ]
        
        # 인스턴스 파일
        instance_files = list((ONTOLOGY_DIR / "instances").glob("*.ttl"))
        
        all_files = core_files + instance_files
        
        for ttl_file in all_files:
            if ttl_file.exists():
                try:
                    self.graph.parse(ttl_file, format="turtle")
                    print(f"  ✅ {ttl_file.name}")
                except Exception as e:
                    print(f"  ⚠️  {ttl_file.name}: {e}")
        
        print(f"\n총 {len(self.graph)} 트리플 로드됨\n")
    
    def execute_query(self, query_text: str) -> List[Dict[str, Any]]:
        """SPARQL 질의 실행"""
        try:
            results = self.graph.query(query_text)
            return [dict(row.asdict()) for row in results]
        except Exception as e:
            print(f"❌ 질의 실행 실패: {e}")
            return []
    
    def run_predefined_query(self, query_name: str):
        """미리 정의된 질의 실행"""
        queries = {
            "critical_open_problems": """
                PREFIX : <http://roarm.ai/ontology#>
                SELECT ?name ?symptom ?created
                WHERE {
                  ?problem a :Problem ;
                           :severity "CRITICAL" ;
                           :status "OPEN" ;
                           :name ?name ;
                           :symptom ?symptom ;
                           :createdAt ?created .
                }
                ORDER BY ?created
            """,
            
            "all_about_pxr": """
                PREFIX : <http://roarm.ai/ontology#>
                SELECT ?entity ?type ?name
                WHERE {
                  {
                    ?entity :relatedTo :pxr_environment_problem ;
                            a ?type .
                    OPTIONAL { ?entity :name ?name }
                  }
                  UNION
                  {
                    :pxr_environment_problem :relatedTo ?entity .
                    ?entity a ?type .
                    OPTIONAL { ?entity :name ?name }
                  }
                  UNION
                  {
                    ?entity :documents :pxr_environment_problem ;
                            a ?type .
                    OPTIONAL { ?entity :name ?name }
                  }
                }
            """,
            
            "pxr_solutions": """
                PREFIX : <http://roarm.ai/ontology#>
                SELECT ?solution ?name ?successRate ?filePath
                WHERE {
                  :pxr_environment_problem :hasSolution ?solution .
                  ?solution :name ?name .
                  OPTIONAL { ?solution :successRate ?successRate }
                  OPTIONAL { ?solution :filePath ?filePath }
                }
                ORDER BY DESC(?successRate)
            """,
            
            "recurring_problems": """
                PREFIX : <http://roarm.ai/ontology#>
                SELECT ?name ?count ?severity ?status
                WHERE {
                  ?problem a :Problem ;
                           :name ?name ;
                           :occurrenceCount ?count ;
                           :severity ?severity ;
                           :status ?status .
                  FILTER (?count > 1)
                }
                ORDER BY DESC(?count)
            """,
            
            "project_status": """
                PREFIX : <http://roarm.ai/ontology#>
                SELECT 
                  (COUNT(DISTINCT ?problem) AS ?totalProblems)
                  (COUNT(DISTINCT ?criticalProblem) AS ?criticalProblems)
                  (COUNT(DISTINCT ?solvedProblem) AS ?solvedProblems)
                  (COUNT(DISTINCT ?openProblem) AS ?openProblems)
                WHERE {
                  ?problem a :Problem .
                  
                  OPTIONAL {
                    ?criticalProblem a :Problem ;
                                     :severity "CRITICAL" .
                  }
                  
                  OPTIONAL {
                    ?solvedProblem a :Problem ;
                                   :status "SOLVED" .
                  }
                  
                  OPTIONAL {
                    ?openProblem a :Problem ;
                                 :status "OPEN" .
                  }
                }
            """,
        }
        
        if query_name not in queries:
            print(f"❌ 알 수 없는 질의: {query_name}")
            print(f"사용 가능한 질의: {', '.join(queries.keys())}")
            sys.exit(1)
        
        print(f"🔍 질의 실행: {query_name}\n")
        results = self.execute_query(queries[query_name])
        
        if not results:
            print("결과 없음")
            return
        
        # 결과 출력
        if query_name == "critical_open_problems":
            print("🔴 CRITICAL 미해결 문제:")
            for i, row in enumerate(results, 1):
                print(f"\n{i}. {row.get('name', 'N/A')}")
                print(f"   증상: {row.get('symptom', 'N/A')}")
                print(f"   발생: {row.get('created', 'N/A')}")
        
        elif query_name == "all_about_pxr":
            print("📦 pxr 관련 엔티티:")
            for row in results:
                entity_type = str(row.get('type', 'N/A')).split('#')[-1]
                entity_name = row.get('name', 'N/A')
                print(f"  • {entity_type}: {entity_name}")
        
        elif query_name == "pxr_solutions":
            print("✅ pxr 문제 해결책:")
            for i, row in enumerate(results, 1):
                print(f"\n{i}. {row.get('name', 'N/A')}")
                if row.get('successRate'):
                    print(f"   성공률: {float(row['successRate']) * 100:.0f}%")
                if row.get('filePath'):
                    print(f"   경로: {row['filePath']}")
        
        elif query_name == "recurring_problems":
            print("🔁 재발 문제:")
            for row in results:
                print(f"  • {row.get('name', 'N/A')} "
                      f"({row.get('count', 'N/A')}회, "
                      f"{row.get('severity', 'N/A')}, "
                      f"{row.get('status', 'N/A')})")
        
        elif query_name == "project_status":
            if results:
                r = results[0]
                print("📊 프로젝트 상태 요약:")
                print(f"  총 문제: {r.get('totalProblems', 0)}")
                print(f"  CRITICAL: {r.get('criticalProblems', 0)}")
                print(f"  해결: {r.get('solvedProblems', 0)}")
                print(f"  미해결: {r.get('openProblems', 0)}")
        
        else:
            # 일반 테이블 출력
            for row in results:
                print(row)
    
    def run_custom_query(self, query_file: Path):
        """커스텀 SPARQL 파일 실행"""
        if not query_file.exists():
            print(f"❌ 질의 파일을 찾을 수 없습니다: {query_file}")
            sys.exit(1)
        
        query_text = query_file.read_text()
        print(f"🔍 커스텀 질의 실행: {query_file.name}\n")
        
        results = self.execute_query(query_text)
        
        if not results:
            print("결과 없음")
            return
        
        # 결과 테이블 출력
        for i, row in enumerate(results, 1):
            print(f"\n{i}.")
            for key, value in row.items():
                print(f"  {key}: {value}")


def main():
    parser = argparse.ArgumentParser(
        description="온톨로지 SPARQL 질의 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  %(prog)s --query critical_open_problems
  %(prog)s --query all_about_pxr
  %(prog)s --query pxr_solutions
  %(prog)s --query recurring_problems
  %(prog)s --query project_status
  %(prog)s --custom-query ontology/queries/diagnostics.sparql
        """
    )
    
    parser.add_argument(
        "--query",
        type=str,
        help="미리 정의된 질의 이름"
    )
    
    parser.add_argument(
        "--custom-query",
        type=Path,
        help="커스텀 SPARQL 파일 경로"
    )
    
    parser.add_argument(
        "--list-queries",
        action="store_true",
        help="사용 가능한 질의 목록 출력"
    )
    
    args = parser.parse_args()
    
    if args.list_queries:
        print("사용 가능한 질의:")
        print("  • critical_open_problems")
        print("  • all_about_pxr")
        print("  • pxr_solutions")
        print("  • recurring_problems")
        print("  • project_status")
        return
    
    if not args.query and not args.custom_query:
        parser.print_help()
        sys.exit(1)
    
    # 온톨로지 로드 및 질의 실행
    oq = OntologyQuery()
    oq.load_ontology()
    
    if args.query:
        oq.run_predefined_query(args.query)
    elif args.custom_query:
        oq.run_custom_query(args.custom_query)


if __name__ == "__main__":
    main()
