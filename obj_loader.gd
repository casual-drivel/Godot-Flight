extends Object
## Minimal runtime OBJ loader (v/vn/f with polygons, no textures needed).
## Builds an immediate-mode ArrayMesh (no index array — indexed surfaces
## silently fail to rasterize on this box's llvmpipe gl_compatibility driver).

static func load_obj(path: String) -> ArrayMesh:
	var f := FileAccess.open(path, FileAccess.READ)
	assert(f != null)
	var lines := f.get_as_text().split("\n", false)
	var vs := PackedVector3Array()
	var vns := PackedVector3Array()
	var out_v := PackedVector3Array()
	var out_n := PackedVector3Array()
	for line in lines:
		var t: String = line.strip_edges()
		if t.begins_with("v "):
			var p := t.split(" ")
			vs.append(Vector3(float(p[1]), float(p[2]), float(p[3])))
		elif t.begins_with("vn "):
			var p := t.split(" ")
			vns.append(Vector3(float(p[1]), float(p[2]), float(p[3])))
		elif t.begins_with("f "):
			var idx_v: Array[int] = []
			var idx_n: Array[int] = []
			for tok in t.substr(2).split(" ", false):
				var parts := tok.split("/")
				idx_v.append(int(parts[0]) - 1)
				var ni: int = -1
				if parts.size() >= 3 and parts[2] != "":
					ni = int(parts[2]) - 1
				idx_n.append(ni)
			# fan-triangulate the polygon
			for i in range(1, idx_v.size() - 1):
				var tri: Array[int] = [idx_v[0], idx_v[i], idx_v[i + 1]]
				var trin: Array[int] = [idx_n[0], idx_n[i], idx_n[i + 1]]
				var a: Vector3 = vs[tri[0]]
				var b: Vector3 = vs[tri[1]]
				var c: Vector3 = vs[tri[2]]
				var geo_n: Vector3 = (b - a).cross(c - a).normalized()
				for k in 3:
					out_v.append(vs[tri[k]])
					var n: Vector3 = geo_n
					if trin[k] >= 0 and trin[k] < vns.size():
						n = vns[trin[k]]
					out_n.append(n)
	var am := ArrayMesh.new()
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = out_v
	arrays[Mesh.ARRAY_NORMAL] = out_n
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return am
