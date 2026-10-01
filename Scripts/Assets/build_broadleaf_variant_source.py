"""既存の幹寸法を保ち，枝順と葉幅の異なる広葉樹の原型を作る．"""
from hashlib import sha256
import math
from pathlib import Path
import random

import build_tree_source as tree


tree.RANDOM = random.Random(84011)
tree.VERTICES.clear()
tree.UVS.clear()
for faces in tree.FACES.values():
	faces.clear()

trunk_direction = tree.unit((0.05, -0.035, 1))
tree.tube((0, 0, 0), trunk_direction, 510, 13, 0, 12, 16)
for branch in range(22):
	height = 138 + branch * 16
	origin = tree.scale(trunk_direction, height)
	angle = branch * 2.399963229728653 + tree.RANDOM.uniform(-0.28, 0.28)
	direction = tree.unit((math.cos(angle), math.sin(angle), tree.RANDOM.uniform(0.06, 0.55)))
	length = (184 - branch * 3.5) * tree.RANDOM.uniform(0.78, 1.17)
	bend = tree.RANDOM.uniform(18, 75)
	tree.tube(origin, direction, length, max(1.2, 4.2 - branch * 0.12), bend, 8, 6)
	for twig in range(5):
		fraction = 0.22 + twig * 0.17
		start = tree.point(origin, direction, length, fraction, bend)
		twig_angle = angle + (-1 if twig % 2 else 1) * tree.RANDOM.uniform(0.43, 1.45)
		twig_direction = tree.unit((math.cos(twig_angle), math.sin(twig_angle), tree.RANDOM.uniform(0.06, 0.7)))
		twig_length = tree.RANDOM.uniform(38, 82)
		tree.tube(start, twig_direction, twig_length, 0.45, 12, 6, 4)
		for pair in range(8):
			for side in (-1, 1):
				location = tree.point(start, twig_direction, twig_length, 0.13 + pair * 0.105, 12)
				leaf_angle = twig_angle + side * tree.RANDOM.uniform(0.48, 1.17)
				leaf_direction = (math.cos(leaf_angle), math.sin(leaf_angle), tree.RANDOM.uniform(-0.55, 0.45))
				tree.leaf(location, leaf_direction, tree.RANDOM.uniform(16, 27),
					tree.RANDOM.uniform(9, 15), tree.RANDOM.uniform(-0.7, 0.7))

lines = ["# Broadleaf sapling crown variant; centimeters; Z up", "mtllib BroadleafSapling.mtl",
	"o BroadleafSaplingVariant"]
lines.extend("v %.8f %.8f %.8f" % vertex for vertex in tree.VERTICES)
lines.extend("vt %.8f %.8f" % uv for uv in tree.UVS)
lines.append("s 1")
for material, faces in tree.FACES.items():
	lines.append("usemtl " + material)
	lines.extend("f " + " ".join(str(index) + "/" + str(index) for index in face) for face in faces)
path = Path(__file__).resolve().parents[2] / "SourceAssets/Vegetation/BroadleafSaplingVariant.obj"
content = ("\n".join(lines) + "\n").encode("utf-8")
if path.exists():
	existing = path.read_bytes()
	if existing != content:
		if sha256(existing).hexdigest().upper() != "F373094202CF1114329C83924880218CCC92446886423DB36360FC6AD67FD0D3":
			raise RuntimeError("既存の樹木原型が旧版と異なる為，上書きしません")
		path.write_bytes(content)
else:
	path.write_bytes(content)
print({"vertices": len(tree.VERTICES), "triangles": {key: len(value) for key, value in tree.FACES.items()},
	"bytes": len(content)})
