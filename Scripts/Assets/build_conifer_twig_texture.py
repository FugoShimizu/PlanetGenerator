"""Poly Haven の針葉枝写真から透明な樹冠片を再現可能な形で作成する"""
from collections import deque
import hashlib
from pathlib import Path
from PIL import Image, ImageChops

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "SourceAssets/Vegetation/ConiferTwig"
EXPECTED = {
	"diff": "e649f0f9bc74d1af5f1a45eaf2fe7a29adaaddfab251f746b983da301e4b80e1",
	"alpha": "bee10734c529a874570da6e4091758de5cc7dee17a359eeacd9c67fde1cb0947",
	"mask": "780336d808718212890ae857fc66a0d1acbbf48c744877527ef195827f2c9fe7"
}
IMAGES = {}
for kind, digest in EXPECTED.items():
	path = SOURCE / ("twig_" + kind + ".jpg")
	if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
		raise RuntimeError("The conifer twig source changed: " + str(path))
	IMAGES[kind] = Image.open(path).convert("RGB" if kind == "diff" else "L")
	if IMAGES[kind].size != (1024, 1024):
		raise RuntimeError("The conifer twig source has an unexpected size")

CROP = (660, 340, 980, 540)
COLOR = IMAGES["diff"].crop(CROP)
ALPHA = ImageChops.darker(IMAGES["alpha"], IMAGES["mask"]).crop(CROP)
ALPHA = ALPHA.point(lambda value: 0 if value < 12 else value)
width, height = COLOR.size
pixels = list(COLOR.getdata())
opacity = list(ALPHA.getdata())
queue = deque(index for index, value in enumerate(opacity) if value >= 64)
visited = bytearray(1 if value >= 64 else 0 for value in opacity)
while queue:
	index = queue.popleft()
	x, y = index % width, index // width
	for neighbor in (index - width if y else None, index + width if y + 1 < height else None,
					 index - 1 if x else None, index + 1 if x + 1 < width else None):
		if neighbor is None or visited[neighbor]:
			continue
		visited[neighbor] = 1
		if opacity[neighbor] < 64:
			pixels[neighbor] = pixels[index]
		queue.append(neighbor)
COLOR.putdata(pixels)
RESULT = COLOR.convert("RGBA")
RESULT.putalpha(ALPHA)
OUTPUT = SOURCE / "ConiferTwigCutout.png"
if OUTPUT.exists():
	current = Image.open(OUTPUT).convert("RGBA")
	if current.size != RESULT.size or current.tobytes() != RESULT.tobytes():
		raise RuntimeError("The existing conifer twig cutout differs; refusing to overwrite it")
else:
	RESULT.save(OUTPUT, optimize=True)
print({"size": RESULT.size, "alphaBoundingBox": ALPHA.getbbox(), "bytes": OUTPUT.stat().st_size,
	"sha256": hashlib.sha256(OUTPUT.read_bytes()).hexdigest()})
