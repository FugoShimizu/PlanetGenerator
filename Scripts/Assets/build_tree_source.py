"""葉・枝・幹を持つ軽量な樹木原型を作成する，惑星上の配置は生成しない"""
import math
from pathlib import Path
import random

ROOT = Path(__file__).resolve().parents[2]
RANDOM = random.Random(73021)
VERTICES = []
UVS = []
FACES = {"Bark": [], "Leaves": []}


def add(a: tuple, b: tuple) -> tuple:
	"""二つの三次元位置を加算する
	Args:
		a: 第１の位置
		b: 第２の位置
	Returns:
		加算結果
	"""
	# 加算した位置の返戻
	return tuple(x + y for x, y in zip(a, b))


def scale(a: tuple, amount: float) -> tuple:
	"""三次元ベクトルを拡大縮小する
	Args:
		a: 対象のベクトル
		amount: 倍率
	Returns:
		拡大縮小後の値
	"""
	# 拡大縮小した値の返戻
	return tuple(x * amount for x in a)


def unit(a: tuple) -> tuple:
	"""ベクトルを正規化する
	Args:
		a: 非零のベクトル
	Returns:
		単位ベクトル
	"""
	# 正規化した方向の返戻
	return scale(a, 1 / math.sqrt(sum(x * x for x in a)))


def cross(a: tuple, b: tuple) -> tuple:
	"""二つのベクトルの外積を求める
	Args:
		a: 第１の方向
		b: 第２の方向
	Returns:
		両方に垂直なベクトル
	"""
	# 外積の返戻
	return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def point(origin: tuple, direction: tuple, length: float, fraction: float, bend: float) -> tuple:
	"""上向きに緩く曲がる枝の中心を求める
	Args:
		origin: 枝の根本
		direction: 初期方向
		length: 枝の長さ
		fraction: 根本からの割合
		bend: 先端の上向き変位
	Returns:
		枝の中心位置
	"""
	# 中心位置の返戻
	return add(add(origin, scale(direction, length * fraction)), (0, 0, bend * fraction * fraction))


def tube(origin: tuple, direction: tuple, length: float, radius: float, bend: float, sides: int, segments: int) -> None:
	"""先端へ細くなる枝又は幹を追加する
	Args:
		origin: 根本
		direction: 初期方向
		length: 長さ
		radius: 根本の半径
		bend: 上向きの曲がり
		sides: 周方向の分割数
		segments: 長さ方向の分割数
	"""
	START = len(VERTICES) + 1
	for ring in range(segments + 1):
		fraction = ring / segments
		center = point(origin, direction, length, fraction, bend)
		tangent = unit(add(scale(direction, length), (0, 0, 2 * bend * fraction)))
		axis = (0, 0, 1) if abs(tangent[2]) < 0.95 else (0, 1, 0)
		x = unit(cross(tangent, axis))
		y = cross(tangent, x)
		r = radius * max(0.025, (1 - fraction) ** 0.75)
		for side in range(sides + 1):
			angle = side / sides * 2 * math.pi
			VERTICES.append(add(center, add(scale(x, math.cos(angle) * r), scale(y, math.sin(angle) * r))))
			UVS.append((side / sides, length * fraction / max(2 * math.pi * radius, 15)))
	for ring in range(segments):
		for side in range(sides):
			a = START + ring * (sides + 1) + side
			b = a + sides + 1
			FACES["Bark"].extend(((a, a + 1, b + 1), (a, b + 1, b)))
	# 終了
	return


def leaf(origin: tuple, direction: tuple, length: float, width: float, twist: float) -> None:
	"""中肋で緩く折れた葉を，確認済み画像の一枚の葉へ対応させる
	Args:
		origin: 葉柄の位置
		direction: 葉先の方向
		length: 葉の長さ
		width: 葉の幅
		twist: 軸周りの角度
	"""
	DIRECTION = unit(direction)
	AXIS = (0, 0, 1) if abs(DIRECTION[2]) < 0.95 else (0, 1, 0)
	SIDE = unit(cross(DIRECTION, AXIS))
	UP = cross(SIDE, DIRECTION)
	ACROSS = add(scale(SIDE, math.cos(twist)), scale(UP, math.sin(twist)))
	NORMAL = cross(ACROSS, DIRECTION)
	START = len(VERTICES) + 1
	for fraction, lateral in ((0, 0), (0.33, -1), (0.33, 0), (0.33, 1), (0.7, -1), (0.7, 0), (0.7, 1), (1, 0)):
		profile = math.sin(math.pi * fraction)
		center = add(origin, scale(DIRECTION, length * fraction))
		vertex = add(center, scale(ACROSS, lateral * width * profile * 0.5))
		vertex = add(vertex, scale(NORMAL, -length * 0.06 * fraction * fraction + (1 - abs(lateral)) * width * profile * 0.08))
		VERTICES.append(vertex)
		# 原画像の右上の葉を使用し，未使用領域の不透明な余白を避ける
		UVS.append((0.701 + 0.122 * fraction + lateral * 0.024 * profile,
			1 - (0.292 - 0.120 * fraction + lateral * 0.024 * profile)))
	for face in ((0, 1, 2), (0, 2, 3), (1, 4, 5), (1, 5, 2), (2, 5, 6), (2, 6, 3), (4, 7, 5), (5, 7, 6)):
		FACES["Leaves"].append(tuple(START + index for index in reversed(face)))
	# 終了
	return


TRUNK_DIRECTION = unit((0.05, -0.035, 1))
tube((0, 0, 0), TRUNK_DIRECTION, 510, 13, 0, 12, 16)
for branch in range(18):
	height = 150 + branch * 18
	origin = scale(TRUNK_DIRECTION, height)
	angle = branch * 2.399963229728653 + RANDOM.uniform(-0.15, 0.15)
	direction = unit((math.cos(angle), math.sin(angle), RANDOM.uniform(0.1, 0.45)))
	length = (205 - branch * 5) * RANDOM.uniform(0.85, 1.1)
	bend = RANDOM.uniform(25, 65)
	tube(origin, direction, length, max(1.2, 4.5 - branch * 0.14), bend, 8, 6)
	for twig in range(6):
		fraction = 0.24 + twig * 0.145
		start = point(origin, direction, length, fraction, bend)
		twig_angle = angle + (-1 if twig % 2 else 1) * RANDOM.uniform(0.5, 1.3)
		twig_direction = unit((math.cos(twig_angle), math.sin(twig_angle), RANDOM.uniform(0.1, 0.65)))
		twig_length = RANDOM.uniform(42, 80)
		tube(start, twig_direction, twig_length, 0.45, 12, 6, 4)
		for pair in range(9):
			for side in (-1, 1):
				location = point(start, twig_direction, twig_length, 0.12 + pair * 0.095, 12)
				leaf_angle = twig_angle + side * RANDOM.uniform(0.5, 1.1)
				leaf_direction = (math.cos(leaf_angle), math.sin(leaf_angle), RANDOM.uniform(-0.5, 0.4))
				leaf(location, leaf_direction, RANDOM.uniform(14, 23), RANDOM.uniform(6, 10), RANDOM.uniform(-0.6, 0.6))
LINES = [
	"# Original broadleaf sapling geometry; centimeters; Z up", "mtllib BroadleafSapling.mtl", "o BroadleafSapling"
]
LINES.extend("v %.8f %.8f %.8f" % vertex for vertex in VERTICES)
LINES.extend("vt %.8f %.8f" % uv for uv in UVS)
LINES.append("s 1")
for material, faces in FACES.items():
	LINES.append("usemtl " + material)
	LINES.extend("f " + " ".join(str(index) + "/" + str(index) for index in face) for face in faces)
SOURCE = ROOT / "SourceAssets/Vegetation/BroadleafSapling.obj"
CONTENT = "\n".join(LINES) + "\n"
if SOURCE.exists() and SOURCE.read_text(encoding="utf-8") != CONTENT:
	raise RuntimeError("既存の樹木原型が異なる為，上書きしません")
if not SOURCE.exists():
	SOURCE.write_text(CONTENT, encoding="utf-8", newline="\n")
print({
	"vertices": len(VERTICES), "triangles": {key: len(value) for key, value in FACES.items()}, "bytes": SOURCE.stat().st_size
})
