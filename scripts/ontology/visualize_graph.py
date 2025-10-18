#!/usr/bin/env python3
"""
온톨로지 지식 그래프 시각화 도구

RDF 온톨로지를 NetworkX 그래프로 변환하여 시각화합니다.
노드 색상/모양으로 엔티티 타입을 구분하고, 관계를 엣지로 표현합니다.

사용법:
    # 전체 그래프 시각화
    python visualize_graph.py --output full_graph.png
    
    # pxr 문제 중심 서브그래프
    python visualize_graph.py --focus pxr_environment_problem --output pxr_subgraph.png
    
    # 특정 클래스만 포함
    python visualize_graph.py --classes Problem,Solution --output problems_solutions.png
"""

import argparse
import sys
from pathlib import Path
from typing import Set, Dict, List, Tuple
from collections import defaultdict

try:
    from rdflib import Graph, Namespace, URIRef, Literal
    from rdflib.namespace import RDF, RDFS, OWL
    import networkx as nx
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
except ImportError as e:
    print(f"❌ 필수 라이브러리 누락: {e}")
    print("설치 명령: pip install rdflib networkx matplotlib")
    sys.exit(1)

# 네임스페이스 정의
ROARM = Namespace("http://roarm.ai/ontology#")

# 색상 팔레트 (엔티티 타입별)
COLOR_MAP = {
    "Problem": "#FF6B6B",           # 빨강 (문제)
    "EnvironmentProblem": "#FF6B6B",
    "PhysicsProblem": "#FF8C8C",
    "Solution": "#51CF66",          # 초록 (해결책)
    "Document": "#4DABF7",          # 파랑 (문서)
    "Script": "#FFD43B",            # 노랑 (스크립트)
    "PreflightCheck": "#9775FA",    # 보라 (검사)
    "Resource": "#868E96",          # 회색 (기타 리소스)
    "USDFile": "#ADB5BD",
    "RobotModel": "#ADB5BD",
    "DevelopmentPhase": "#20C997",  # 청록 (개발 단계)
}

# 노드 모양 (엔티티 타입별)
SHAPE_MAP = {
    "Problem": "o",          # 원
    "Solution": "s",         # 사각형
    "Document": "^",         # 삼각형
    "Script": "D",           # 다이아몬드
    "PreflightCheck": "p",   # 오각형
    "DevelopmentPhase": "h", # 육각형
}


class OntologyVisualizer:
    """온톨로지 시각화 클래스"""
    
    def __init__(self, ontology_dir: Path):
        """
        Args:
            ontology_dir: ontology/ 디렉토리 경로
        """
        self.ontology_dir = ontology_dir
        self.graph = Graph()
        self.nx_graph = nx.DiGraph()
        self.entity_types: Dict[str, str] = {}  # entity_id -> type
        
    def load_ontology(self) -> int:
        """ontology/ 디렉토리의 모든 .ttl 파일 로드"""
        triple_count = 0
        
        # Core ontology
        core_file = self.ontology_dir / "roarm_domain.ttl"
        if core_file.exists():
            self.graph.parse(core_file, format="turtle")
            print(f"✓ {core_file.name} 로드됨")
        
        # Instances
        instances_dir = self.ontology_dir / "instances"
        if instances_dir.exists():
            for ttl_file in instances_dir.glob("*.ttl"):
                self.graph.parse(ttl_file, format="turtle")
                print(f"✓ {ttl_file.name} 로드됨")
        
        triple_count = len(self.graph)
        print(f"\n총 {triple_count} 트리플 로드됨\n")
        return triple_count
    
    def extract_type(self, entity_uri: URIRef) -> str:
        """엔티티의 가장 구체적인 타입 추출"""
        types = list(self.graph.objects(entity_uri, RDF.type))
        
        # OWL/RDF 내부 타입 제외
        user_types = [t for t in types if not str(t).startswith(str(OWL)) and not str(t).startswith(str(RDFS))]
        
        if not user_types:
            return "Resource"
        
        # 가장 구체적인 타입 선택 (Problem보다 EnvironmentProblem 우선)
        type_names = [str(t).split('#')[-1] for t in user_types]
        
        # 우선순위: 하위 클래스 > 상위 클래스
        priority = ["EnvironmentProblem", "PhysicsProblem", "Problem", 
                   "Solution", "Document", "Script", "PreflightCheck", 
                   "DevelopmentPhase", "USDFile", "RobotModel"]
        
        for ptype in priority:
            if ptype in type_names:
                return ptype
        
        return type_names[0] if type_names else "Resource"
    
    def build_networkx_graph(self, focus_entity: str = None, depth: int = 2):
        """RDF 그래프를 NetworkX 그래프로 변환"""
        print("NetworkX 그래프 구축 중...")
        
        # 시작 엔티티 설정
        if focus_entity:
            start_entities = {ROARM[focus_entity]}
        else:
            # 모든 엔티티 포함
            start_entities = set(self.graph.subjects())
        
        # BFS로 depth만큼 탐색
        visited: Set[URIRef] = set()
        to_visit: List[Tuple[URIRef, int]] = [(e, 0) for e in start_entities]
        
        while to_visit:
            entity, current_depth = to_visit.pop(0)
            
            if entity in visited:
                continue
            
            # depth 체크는 관계 추가 전에만
            if current_depth > depth:
                continue
            
            visited.add(entity)
            
            # 엔티티 ID 추출 (URI에서 로컬명)
            entity_id = str(entity).split('#')[-1]
            
            # 타입 결정
            entity_type = self.extract_type(entity)
            self.entity_types[entity_id] = entity_type
            
            # 노드 추가
            self.nx_graph.add_node(entity_id, entity_type=entity_type)
            
            # 관계 탐색 (나가는 방향: entity -> obj)
            for pred, obj in self.graph.predicate_objects(entity):
                # Literal이나 내부 속성 제외
                if isinstance(obj, Literal) or str(pred) in [str(RDF.type), str(RDFS.label)]:
                    continue
                
                # URI 객체만 처리
                if isinstance(obj, URIRef) and str(obj).startswith(str(ROARM)):
                    obj_id = str(obj).split('#')[-1]
                    pred_name = str(pred).split('#')[-1]
                    
                    # 목적지 노드도 추가
                    if obj_id not in self.entity_types:
                        obj_type = self.extract_type(obj)
                        self.entity_types[obj_id] = obj_type
                        self.nx_graph.add_node(obj_id, entity_type=obj_type)
                    
                    # 엣지 추가
                    self.nx_graph.add_edge(entity_id, obj_id, relation=pred_name)
                    
                    # 탐색 큐에 추가
                    to_visit.append((obj, current_depth + 1))
            
            # 들어오는 방향도 탐색 (obj -> entity)
            for subj, pred in self.graph.subject_predicates(entity):
                if str(pred) in [str(RDF.type), str(RDFS.label)]:
                    continue
                
                if isinstance(subj, URIRef) and str(subj).startswith(str(ROARM)):
                    subj_id = str(subj).split('#')[-1]
                    pred_name = str(pred).split('#')[-1]
                    
                    # 소스 노드도 추가
                    if subj_id not in self.entity_types:
                        subj_type = self.extract_type(subj)
                        self.entity_types[subj_id] = subj_type
                        self.nx_graph.add_node(subj_id, entity_type=subj_type)
                    
                    # 엣지 추가 (역방향)
                    self.nx_graph.add_edge(subj_id, entity_id, relation=pred_name)
                    
                    # 탐색 큐에 추가
                    to_visit.append((subj, current_depth + 1))
        
        print(f"✓ 노드 {self.nx_graph.number_of_nodes()}개, 엣지 {self.nx_graph.number_of_edges()}개 구축\n")
    
    def draw_graph(self, output_file: Path, layout: str = "spring", figsize: Tuple[int, int] = (20, 16)):
        """NetworkX 그래프를 Matplotlib으로 시각화"""
        print(f"그래프 시각화 중 (레이아웃: {layout})...")
        
        # Figure 생성
        fig, ax = plt.subplots(figsize=figsize)
        
        # 레이아웃 선택
        if layout == "spring":
            pos = nx.spring_layout(self.nx_graph, k=2, iterations=50, seed=42)
        elif layout == "circular":
            pos = nx.circular_layout(self.nx_graph)
        elif layout == "kamada":
            pos = nx.kamada_kawai_layout(self.nx_graph)
        else:
            pos = nx.spring_layout(self.nx_graph, seed=42)
        
        # 타입별 노드 그룹화
        nodes_by_type = defaultdict(list)
        for node in self.nx_graph.nodes():
            node_type = self.entity_types.get(node, "Resource")
            nodes_by_type[node_type].append(node)
        
        # 타입별로 노드 그리기
        for node_type, nodes in nodes_by_type.items():
            color = COLOR_MAP.get(node_type, "#868E96")
            shape = SHAPE_MAP.get(node_type, "o")
            
            nx.draw_networkx_nodes(
                self.nx_graph, pos,
                nodelist=nodes,
                node_color=color,
                node_shape=shape,
                node_size=1500,
                alpha=0.9,
                ax=ax
            )
        
        # 엣지 그리기
        nx.draw_networkx_edges(
            self.nx_graph, pos,
            edge_color="#ADB5BD",
            arrows=True,
            arrowsize=20,
            arrowstyle="->",
            width=2,
            alpha=0.6,
            ax=ax
        )
        
        # 라벨 그리기
        nx.draw_networkx_labels(
            self.nx_graph, pos,
            font_size=9,
            font_weight="bold",
            font_color="#212529",
            ax=ax
        )
        
        # 엣지 라벨 (관계명)
        edge_labels = nx.get_edge_attributes(self.nx_graph, "relation")
        nx.draw_networkx_edge_labels(
            self.nx_graph, pos,
            edge_labels,
            font_size=7,
            font_color="#495057",
            ax=ax
        )
        
        # 범례 생성
        legend_elements = []
        for node_type in sorted(set(self.entity_types.values())):
            color = COLOR_MAP.get(node_type, "#868E96")
            shape = SHAPE_MAP.get(node_type, "o")
            
            # Matplotlib marker 변환
            marker_map = {"o": "o", "s": "s", "^": "^", "D": "D", "p": "p", "h": "h"}
            marker = marker_map.get(shape, "o")
            
            legend_elements.append(
                mpatches.Patch(facecolor=color, edgecolor="black", label=node_type)
            )
        
        ax.legend(handles=legend_elements, loc="upper left", fontsize=10, framealpha=0.9)
        
        # 제목
        ax.set_title("RoArm M3 Isaac Sim 온톨로지 지식 그래프", fontsize=16, fontweight="bold", pad=20)
        ax.axis("off")
        
        # 저장
        plt.tight_layout()
        plt.savefig(output_file, dpi=150, bbox_inches="tight", facecolor="white")
        print(f"✅ 시각화 저장: {output_file}\n")
        
        plt.close()
    
    def print_statistics(self):
        """그래프 통계 출력"""
        print("=" * 60)
        print("📊 그래프 통계")
        print("=" * 60)
        print(f"노드 수: {self.nx_graph.number_of_nodes()}")
        print(f"엣지 수: {self.nx_graph.number_of_edges()}")
        print(f"\n타입별 노드 분포:")
        
        type_counts = defaultdict(int)
        for node_type in self.entity_types.values():
            type_counts[node_type] += 1
        
        for node_type, count in sorted(type_counts.items(), key=lambda x: -x[1]):
            print(f"  • {node_type}: {count}")
        
        # 연결성 분석
        if self.nx_graph.number_of_nodes() > 0:
            print(f"\n연결 컴포넌트 수: {nx.number_weakly_connected_components(self.nx_graph)}")
            
            # 가장 중요한 노드 (in-degree 기준)
            in_degrees = dict(self.nx_graph.in_degree())
            if in_degrees:
                top_nodes = sorted(in_degrees.items(), key=lambda x: -x[1])[:5]
                print(f"\n가장 많이 참조되는 노드 (Top 5):")
                for node, degree in top_nodes:
                    node_type = self.entity_types.get(node, "Unknown")
                    print(f"  • {node} ({node_type}): {degree}개 참조")
        
        print("=" * 60)
        print()


def main():
    parser = argparse.ArgumentParser(
        description="RDF 온톨로지 지식 그래프 시각화",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예제:
  # 전체 그래프
  python visualize_graph.py --output full_graph.png
  
  # pxr 문제 중심 서브그래프
  python visualize_graph.py --focus pxr_environment_problem --depth 2 --output pxr_subgraph.png
  
  # Kamada-Kawai 레이아웃으로 렌더링
  python visualize_graph.py --layout kamada --output ontology_kamada.png
        """
    )
    
    parser.add_argument(
        "--ontology-dir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "ontology",
        help="ontology/ 디렉토리 경로 (기본: ../../ontology)"
    )
    parser.add_argument(
        "--focus",
        type=str,
        default=None,
        help="특정 엔티티 중심으로 서브그래프 추출 (예: pxr_environment_problem)"
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=2,
        help="--focus 사용 시 탐색 깊이 (기본: 2)"
    )
    parser.add_argument(
        "--layout",
        choices=["spring", "circular", "kamada"],
        default="spring",
        help="그래프 레이아웃 알고리즘 (기본: spring)"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("ontology_graph.png"),
        help="출력 PNG 파일명 (기본: ontology_graph.png)"
    )
    parser.add_argument(
        "--figsize",
        type=str,
        default="20,16",
        help="Figure 크기 (width,height in inches, 기본: 20,16)"
    )
    
    args = parser.parse_args()
    
    # Figure 크기 파싱
    try:
        width, height = map(int, args.figsize.split(','))
        figsize = (width, height)
    except:
        print(f"⚠️  잘못된 figsize 형식: {args.figsize}, 기본값 (20,16) 사용")
        figsize = (20, 16)
    
    # 온톨로지 디렉토리 검증
    if not args.ontology_dir.exists():
        print(f"❌ 온톨로지 디렉토리를 찾을 수 없습니다: {args.ontology_dir}")
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("🎨 RoArm M3 온톨로지 시각화 도구")
    print("=" * 60)
    print()
    
    # 시각화 실행
    visualizer = OntologyVisualizer(args.ontology_dir)
    
    # 1. 온톨로지 로드
    triple_count = visualizer.load_ontology()
    if triple_count == 0:
        print("❌ 온톨로지 파일이 비어있습니다.")
        sys.exit(1)
    
    # 2. NetworkX 그래프 구축
    visualizer.build_networkx_graph(focus_entity=args.focus, depth=args.depth)
    
    # 3. 시각화
    visualizer.draw_graph(args.output, layout=args.layout, figsize=figsize)
    
    # 4. 통계 출력
    visualizer.print_statistics()
    
    print(f"✨ 완료! 시각화 파일: {args.output.absolute()}")


if __name__ == "__main__":
    main()
