"""固定乱数からセンチメートル単位の軽量な針葉樹を作成する"""
import math
from pathlib import Path
import random

ROOT = Path(__file__).resolve().parents[2]
RANDOM = random.Random(91307)
VERTICES = []
UVS = []
FACES = {"Bark": [], "Needles": []}


def unit(vector: tuple) -> tuple:
	"""零でない三次元方向を単位化する

	Args:
		vector: 零でない三次元方向

	Returns:
		単位方向
	"""
	length = math.sqrt(sum(value * value for value in vector))
	return tuple(value / length for value in vector)


def add(a: tuple, b: tuple) -> tuple:
	"""三次元方向を加算する

	Args:
		a: 第一の方向
		b: 第二の方向

	Returns:
		加算した方向
	"""
	return tuple(x + y for x, y in zip(a, b))


def mul(vector: tuple, amount: float) -> tuple:
	"""三次元方向を倍率で伸縮する

	Args:
		vector: 元の方向
		amount: 倍率

	Returns:
		伸縮した方向
	"""
	return tuple(value * amount for value in vector)


def cross(a: tuple, b: tuple) -> tuple:
	"""三次元方向の外積を算出する

	Args:
		a: 第一の方向
		b: 第二の方向

	Returns:
		両方向に垂直な方向
	"""
	return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def tapered_tube(start: tuple, end: tuple, base_radius: float, tip_radius: float, sides: int) -> None:
	"""先細りの幹又は枝の樹皮面を追加する

	Args:
		start: 根元の位置
		end: 先端の位置
		base_radius: 根元の半径，単位はセンチメートル
		tip_radius: 先端の半径，単位はセンチメートル
		sides: 円周方向の分割数
	"""
	axis = unit(tuple(end[i] - start[i] for i in range(3)))
	basis = (0, 0, 1) if abs(axis[2]) < 0.9 else (0, 1, 0)
	x = unit(cross(axis, basis))
	y = cross(axis, x)
	first = len(VERTICES) + 1
	for ring, center, radius in ((0, start, base_radius), (1, end, tip_radius)):
		for side in range(sides + 1):
			angle = 2 * math.pi * side / sides
			VERTICES.append(add(center, add(mul(x, radius * math.cos(angle)), mul(y, radius * math.sin(angle)))))
			UVS.append((side / sides, ring * 0.75))
	for side in range(sides):
		a = first + side
		b = a + sides + 1
		FACES["Bark"].extend(((a, a + 1, b + 1), (a, b + 1, b)))
	return


def needle_card(start: tuple, end: tuple, radius: float) -> None:
	"""針葉写真を貼る三方向の枝片面を追加する

	Args:
		start: 枝片の根元
		end: 枝片の先端
		radius: 面の半幅，単位はセンチメートル
	"""
	axis = unit(tuple(end[i] - start[i] for i in range(3)))
	basis = (0, 0, 1) if abs(axis[2]) < 0.9 else (0, 1, 0)
	x = unit(cross(axis, basis))
	y = cross(axis, x)
	for across in (x, add(mul(x, 0.5), mul(y, math.sqrt(3) / 2)), add(mul(x, -0.5), mul(y, math.sqrt(3) / 2))):
		first = len(VERTICES) + 1
		for point, uv in ((add(start, mul(across, -radius)), (0, 0)),
							(add(start, mul(across, radius)), (0, 1)),
							(add(end, mul(across, radius)), (1, 1)),
							(add(end, mul(across, -radius)), (1, 0))):
			VERTICES.append(point)
			UVS.append(uv)
		FACES["Needles"].extend(((first, first + 1, first + 2), (first, first + 2, first + 3)))
	return


for segment in range(14):
	z = 680 * segment / 14
	next_z = 680 * (segment + 1) / 14
	taper = lambda height: max(1.5, 17 * (1 - height / 710) ** 0.8)
	tapered_tube((0, 0, z), (0, 0, next_z), taper(z), taper(next_z), 10)
for level in range(18):
	height = 115 + 30 * level
	length = 225 * (1 - level / 18) ** 0.85
	for branch in range(6):
		angle = 2 * math.pi * (branch / 6 + (level % 2) / 12) + RANDOM.uniform(-0.07, 0.07)
		azimuth = (math.cos(angle), math.sin(angle))
		branch_length = length * RANDOM.uniform(0.86, 1.08)
		drop = 24 * (1 - level / 18) + RANDOM.uniform(-5, 5)
		start = (azimuth[0] * 7, azimuth[1] * 7, height)
		end = (azimuth[0] * branch_length, azimuth[1] * branch_length, height - drop)
		tapered_tube(start, end, max(1.5, 4.5 * (1 - level / 20)), 0.6, 5)
		for twig in range(5):
			fraction = 0.18 + twig * 0.17
			origin = add(start, mul(tuple(end[i] - start[i] for i in range(3)), fraction))
			side = 1 if twig % 2 else -1
			twig_angle = angle + side * RANDOM.uniform(0.55, 1.05)
			twig_direction = unit((math.cos(twig_angle), math.sin(twig_angle), RANDOM.uniform(-0.05, 0.18)))
			twig_length = branch_length * (0.35 - 0.12 * fraction) * RANDOM.uniform(0.85, 1.1)
			tip = add(origin, mul(twig_direction, twig_length))
			needle_card(origin, tip, 1.2 * (12 + 5 * (1 - fraction)) * (1 - level / 24))
tapered_tube((0, 0, 660), (0, 0, 710), 2.5, 0.3, 6)

LINES = ["# Original conifer geometry; centimeters; Z up", "mtllib ConiferSapling.mtl", "o ConiferSapling"]
LINES.extend("v %.8f %.8f %.8f" % vertex for vertex in VERTICES)
LINES.extend("vt %.8f %.8f" % uv for uv in UVS)
LINES.append("s 1")
for material, faces in FACES.items():
	LINES.append("usemtl " + material)
	LINES.extend("f " + " ".join(str(index) + "/" + str(index) for index in face) for face in faces)
SOURCE = ROOT / "SourceAssets/Vegetation/ConiferSapling.obj"
CONTENT = "\n".join(LINES) + "\n"
if SOURCE.exists() and SOURCE.read_text(encoding="utf-8") != CONTENT:
	import hashlib
	previous = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
	if previous != "e1c0b0eb5ea5e262a364f28686e5404ef9993790d6a54a9517c75b74441b6bde":
		raise RuntimeError("Existing conifer geometry differs; refusing to overwrite it")
	SOURCE.write_bytes(CONTENT.encode("utf-8"))
elif not SOURCE.exists():
	SOURCE.write_bytes(CONTENT.encode("utf-8"))
print({"vertices": len(VERTICES), "triangles": {key: len(value) for key, value in FACES.items()},
	"bytes": SOURCE.stat().st_size})
