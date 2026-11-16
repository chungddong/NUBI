"""PowerPoint로 덱을 열어 슬라이드마다 PNG로 내보낸다 (화면 점검용, Windows + PowerPoint 필요).

실행: py render.py [출력 폴더]   (기본: ./render)
"""
import sys
from pathlib import Path

import win32com.client

HERE = Path(__file__).resolve().parent
DECK = HERE / "LoadGoesOn_공학페스티벌_전시.pptx"
out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "render"
out.mkdir(parents=True, exist_ok=True)
for old in out.glob("slide-*.png"):
    old.unlink()

app = win32com.client.DispatchEx("PowerPoint.Application")
try:
    pres = app.Presentations.Open(str(DECK), True, False, False)  # 읽기 전용, 창 없이
    for s in pres.Slides:
        s.Export(str(out / f"slide-{s.SlideIndex:02d}.png"), "PNG", 1600, 900)
    print("slides:", pres.Slides.Count)
    pres.Close()
finally:
    app.Quit()
for p in sorted(out.glob("slide-*.png")):
    print(p)
