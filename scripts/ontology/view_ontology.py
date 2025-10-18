#!/usr/bin/env python3
"""
온톨로지 시각화 웹 뷰어

생성된 PNG 이미지를 브라우저에서 확인할 수 있는 간단한 웹 서버입니다.

사용법:
    python view_ontology.py
    
    # 포트 변경
    python view_ontology.py --port 8080
"""

import argparse
import sys
from pathlib import Path
from http.server import HTTPServer, SimpleHTTPRequestHandler
import webbrowser
import threading
import time


class OntologyHTMLGenerator:
    """온톨로지 시각화 HTML 생성"""
    
    def __init__(self, docs_dir: Path):
        self.docs_dir = docs_dir
        self.ontology_dir = docs_dir / "ontology"
        
    def generate_html(self) -> str:
        """메인 HTML 페이지 생성"""
        # 이미지 파일 찾기
        images = []
        if self.ontology_dir.exists():
            for img in self.ontology_dir.glob("*.png"):
                images.append({
                    "name": img.name,
                    "path": f"ontology/{img.name}",
                    "size": img.stat().st_size,
                })
        
        # 이미지 카드 생성
        image_cards = ""
        for img in images:
            size_mb = img['size'] / (1024 * 1024)
            image_cards += f"""
            <div class="image-card">
                <h3>{img['name']}</h3>
                <p class="image-info">크기: {size_mb:.2f} MB</p>
                <a href="{img['path']}" target="_blank" class="view-btn">
                    🔍 원본 크기로 보기
                </a>
                <div class="image-preview">
                    <img src="{img['path']}" alt="{img['name']}" />
                </div>
            </div>
            """
        
        if not images:
            image_cards = """
            <div class="no-images">
                <p>⚠️ 시각화 이미지가 없습니다.</p>
                <p>다음 명령으로 생성하세요:</p>
                <pre>python scripts/ontology/visualize_graph.py --output docs/ontology/graph.png</pre>
            </div>
            """
        
        html = f"""<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RoArm M3 온톨로지 시각화</title>
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}
        
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }}
        
        .container {{
            max-width: 1400px;
            margin: 0 auto;
        }}
        
        header {{
            background: white;
            border-radius: 10px;
            padding: 30px;
            margin-bottom: 30px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
        }}
        
        h1 {{
            color: #333;
            margin-bottom: 10px;
            font-size: 2.5em;
        }}
        
        .subtitle {{
            color: #666;
            font-size: 1.1em;
        }}
        
        .stats {{
            display: flex;
            gap: 20px;
            margin-top: 20px;
            flex-wrap: wrap;
        }}
        
        .stat-box {{
            background: #f8f9fa;
            padding: 15px 25px;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }}
        
        .stat-box .label {{
            color: #666;
            font-size: 0.9em;
            margin-bottom: 5px;
        }}
        
        .stat-box .value {{
            color: #333;
            font-size: 1.5em;
            font-weight: bold;
        }}
        
        .image-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(600px, 1fr));
            gap: 30px;
        }}
        
        .image-card {{
            background: white;
            border-radius: 10px;
            padding: 25px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
            transition: transform 0.3s ease;
        }}
        
        .image-card:hover {{
            transform: translateY(-5px);
        }}
        
        .image-card h3 {{
            color: #333;
            margin-bottom: 10px;
            font-size: 1.3em;
        }}
        
        .image-info {{
            color: #666;
            margin-bottom: 15px;
            font-size: 0.9em;
        }}
        
        .view-btn {{
            display: inline-block;
            background: #667eea;
            color: white;
            padding: 10px 20px;
            border-radius: 5px;
            text-decoration: none;
            margin-bottom: 20px;
            transition: background 0.3s ease;
        }}
        
        .view-btn:hover {{
            background: #5568d3;
        }}
        
        .image-preview {{
            border: 2px solid #e9ecef;
            border-radius: 8px;
            overflow: hidden;
            background: #f8f9fa;
        }}
        
        .image-preview img {{
            width: 100%;
            height: auto;
            display: block;
            cursor: pointer;
        }}
        
        .no-images {{
            background: white;
            border-radius: 10px;
            padding: 40px;
            text-align: center;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
        }}
        
        .no-images p {{
            margin-bottom: 15px;
            color: #666;
            font-size: 1.1em;
        }}
        
        .no-images pre {{
            background: #f8f9fa;
            padding: 15px;
            border-radius: 5px;
            display: inline-block;
            text-align: left;
            color: #333;
            margin-top: 10px;
        }}
        
        .tools {{
            background: white;
            border-radius: 10px;
            padding: 25px;
            margin-bottom: 30px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
        }}
        
        .tools h2 {{
            color: #333;
            margin-bottom: 15px;
        }}
        
        .tools ul {{
            list-style: none;
            padding-left: 0;
        }}
        
        .tools li {{
            padding: 10px 0;
            color: #666;
            border-bottom: 1px solid #e9ecef;
        }}
        
        .tools li:last-child {{
            border-bottom: none;
        }}
        
        .tools code {{
            background: #f8f9fa;
            padding: 2px 8px;
            border-radius: 3px;
            font-family: 'Courier New', monospace;
            color: #e83e8c;
        }}
        
        footer {{
            text-align: center;
            color: white;
            margin-top: 40px;
            padding: 20px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>🧠 RoArm M3 온톨로지 시각화</h1>
            <p class="subtitle">Isaac Sim 5.0 프로젝트 지식 그래프</p>
            
            <div class="stats">
                <div class="stat-box">
                    <div class="label">총 트리플</div>
                    <div class="value">781</div>
                </div>
                <div class="stat-box">
                    <div class="label">문제 추적</div>
                    <div class="value">3개</div>
                </div>
                <div class="stat-box">
                    <div class="label">CRITICAL 해결</div>
                    <div class="value">100%</div>
                </div>
                <div class="stat-box">
                    <div class="label">시각화</div>
                    <div class="value">{len(images)}개</div>
                </div>
            </div>
        </header>
        
        <div class="tools">
            <h2>🔧 사용 가능한 도구</h2>
            <ul>
                <li>📊 <code>query_ontology.py</code> - SPARQL 질의 실행</li>
                <li>🎨 <code>visualize_graph.py</code> - 지식 그래프 PNG 생성</li>
                <li>➕ <code>add_problem.py</code> - 신규 문제 등록</li>
            </ul>
        </div>
        
        <div class="image-grid">
            {image_cards}
        </div>
        
        <footer>
            <p>RoArm M3 + Isaac Sim 5.0 | 온톨로지 기반 지식 관리 시스템</p>
            <p style="margin-top: 10px; font-size: 0.9em;">Press Ctrl+C in terminal to stop server</p>
        </footer>
    </div>
    
    <script>
        // 이미지 클릭 시 새 탭에서 열기
        document.querySelectorAll('.image-preview img').forEach(img => {{
            img.addEventListener('click', () => {{
                window.open(img.src, '_blank');
            }});
        }});
    </script>
</body>
</html>
"""
        return html


class OntologyHTTPHandler(SimpleHTTPRequestHandler):
    """커스텀 HTTP 핸들러"""
    
    docs_dir = None  # 클래스 변수로 설정
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(self.docs_dir), **kwargs)
    
    def do_GET(self):
        """GET 요청 처리"""
        if self.path == '/' or self.path == '/index.html':
            # 메인 HTML 생성
            generator = OntologyHTMLGenerator(self.docs_dir)
            html = generator.generate_html()
            
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(html.encode('utf-8'))
        else:
            # 정적 파일 (이미지 등)
            super().do_GET()
    
    def log_message(self, format, *args):
        """로그 메시지 포맷"""
        print(f"[{self.address_string()}] {format % args}")


def open_browser(url: str, delay: float = 1.0):
    """브라우저 자동 열기 (지연 후)"""
    time.sleep(delay)
    webbrowser.open(url)
    print(f"\n✅ 브라우저에서 {url} 열림")


def main():
    parser = argparse.ArgumentParser(
        description="온톨로지 시각화 웹 뷰어",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="웹 서버 포트 (기본: 8000)"
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="브라우저 자동 열기 비활성화"
    )
    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=Path(__file__).parent.parent.parent / "docs",
        help="docs/ 디렉토리 경로"
    )
    
    args = parser.parse_args()
    
    # docs 디렉토리 확인
    if not args.docs_dir.exists():
        print(f"❌ docs 디렉토리를 찾을 수 없습니다: {args.docs_dir}")
        sys.exit(1)
    
    # HTTP 서버 설정
    OntologyHTTPHandler.docs_dir = args.docs_dir  # 클래스 변수 설정
    
    server = HTTPServer(('localhost', args.port), OntologyHTTPHandler)
    url = f"http://localhost:{args.port}"
    
    print("\n" + "=" * 60)
    print("🌐 온톨로지 시각화 웹 뷰어 시작")
    print("=" * 60)
    print(f"\n📍 URL: {url}")
    print(f"📁 Docs: {args.docs_dir.absolute()}")
    print("\n🔧 사용법:")
    print("  • 브라우저에서 위 URL 접속")
    print("  • 이미지 클릭으로 확대 보기")
    print("  • Ctrl+C로 서버 종료")
    print("\n" + "=" * 60 + "\n")
    
    # 브라우저 자동 열기
    if not args.no_browser:
        browser_thread = threading.Thread(target=open_browser, args=(url,))
        browser_thread.daemon = True
        browser_thread.start()
    
    try:
        print("🚀 서버 실행 중... (Ctrl+C로 종료)")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\n✅ 서버 종료")
        server.shutdown()


if __name__ == "__main__":
    main()
