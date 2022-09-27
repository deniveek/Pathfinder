import numpy as np
from operator import xor
from functools import reduce


def intersects(s0p0, s0p1, s1p0, s1p1):
	return xor(isRigth(s0p0, s0p1, s1p0), isRigth(s0p0, s0p1, s1p1)) and xor(isRigth(s1p0, s1p1, s0p0), isRigth(s1p0, s1p1, s0p1))


def segLenSq(x1, y1, x2, y2):
	return (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)


def segLenSqPt(p1, p2):
	return segLenSq(p1[0], p1[1], p2[0], p2[1])


def samePoints(p1, p2):
	return p1[0] == p2[0] and p1[1] == p2[1]


def isRigth(p0, p1, p):
	ab = p1 - p0
	ac = p - p0
	return True if np.cross(ac, ab) > 0 else False


def BBox(obstacles):
		def min_x(cur, points):
			new_val = np.amin(points, 0)[0]
			return cur if cur < new_val else new_val

		def min_y(cur, points):
			new_val = np.amin(points, 1)[1]
			return cur if cur < new_val else new_val

		def max_x(cur, points):
			new_val = np.amax(points, 0)[0]
			return cur if cur > new_val else new_val

		def max_y(cur, points):
			new_val = np.amax(points, 1)[1]
			return cur if cur > new_val else new_val

		minx = reduce(min_x, obstacles, np.amin(obstacles[0], 0)[0])
		miny = reduce(min_y, obstacles, np.amin(obstacles[0], 1)[1])
		maxx = reduce(max_x, obstacles, np.amax(obstacles[0], 0)[0])
		maxy = reduce(max_y, obstacles, np.amax(obstacles[0], 1)[1])

		return [minx, miny, maxx, maxy]


def AugmentBBox(bbox, factor):
    c_x = (bbox[0] + bbox[2])/2
    c_y = (bbox[1] + bbox[3])/2
    dx = (bbox[2]-bbox[0])*factor
    dy = (bbox[2]-bbox[0])*factor
    min_x = c_x - dx/2
    max_x = c_x + dx/2
    min_y = c_y - dy/2
    max_y = c_y + dy/2
    return [min_x, min_y, max_x, max_y]


def MonteCarlo(obstacles, N):
	if len(obstacles) > 0:
		bbox = AugmentBBox(BBox(obstacles), 1.1)
		return np.array([list(a) for a in zip( np.random.random_integers(bbox[0], bbox[2], N) ,np.random.random_integers(bbox[1], bbox[3], N))])
	else:
		return None


def segments(obstacles):
	segs = []
	for wall in obstacles:
		segs.extend([[np.array([s[0][0], s[0][1]]), np.array([s[1][0], s[1][1]])] for s in zip(wall[:-1], wall[1:])])
	return segs


def visible(p0, p1, segs):
	closing = next((s for s in segs if intersects(p0, p1, s[0], s[1])), None)
	return closing is None


class Dijkstra:
	def __init__(self, obstacles, pt_start, pt_finish):
		# points
		mcPoints = MonteCarlo(obstacles, 20)
		if mcPoints is not None:
			self.points = np.append(mcPoints, [pt_start, pt_finish], axis=0)
		else:
			self.points = np.array([pt_start, pt_finish])
		#print(f"points={self.points}")
		self.start_idx  = len(self.points)-2
		self.finish_idx = len(self.points)-1
		#print(f"start_idx={self.start_idx} finish_idx={self.finish_idx}")
		# obstacles segments
		self.segments = segments(obstacles)
		#print(f"segments={self.segments}")
		# visited points and weights
		self.visited = [None for p in self.points]
		self.visited[self.start_idx] = 0
		#print(f"visited={self.visited}")

	def findPath(self, cur_idx, weight):
		#print(f"cur={cur_idx} weight={weight}")
		# Если в эту точку есть более короткий путь - возвращаемся
		if self.visited[cur_idx] is not None and self.visited[cur_idx] < weight:
			#print(f"no way from {cur_idx}")
			return [-1, []]
		else:
			self.visited[cur_idx] = weight

		# координаты текущей точки
		cur_pt = self.points[cur_idx]

		# сразу проверим видимость конечной точки
		finish_pt = self.points[self.finish_idx]

		# Если конечная точка видна - начинаем восстанавливать путь
		if visible(cur_pt, finish_pt, self.segments):
			dW = segLenSq(cur_pt[0], cur_pt[1], finish_pt[0], finish_pt[1])
			if self.visited[self.finish_idx] is None or self.visited[self.finish_idx] > weight+dW:
				self.visited[self.finish_idx] = weight+dW
				return [self.visited[self.finish_idx], [finish_pt, cur_pt]]

		# видимые точки из данной
		adjacent_points = [[idx, segLenSqPt(cur_pt, p)] for idx, p in enumerate(self.points) if cur_idx != idx and visible(cur_pt, p, self.segments)]

		# Смотрим, куда еще не ходили или наш путь короче
		paths = []
		for point in adjacent_points:
			if self.visited[point[0]] is None or self.visited[point[0]] > weight + point[1]:
				new_path = self.findPath(point[0], weight + point[1])
				if new_path[0] != -1:
					paths.append(new_path)

		#print(f"cur_idx: {cur_idx}, paths = {paths}")
		if len(paths) > 0:
			min_path = reduce(lambda cur_min, p: cur_min if cur_min[0] < p[0] else p , paths)
			#print(f"min_path={min_path}, cur_pt = {cur_pt}")
			res_path = min_path[1]
			res_path.append(cur_pt)
			#print(f"res_path={res_path}")
			return [min_path[0], res_path]

		else:	
			#print(f"no way from {cur_idx}")
			return [-1, []]


def findPath(obstacles, start, finish):
	#print("Start findPath")
	d = Dijkstra(obstacles, start, finish)
	path = d.findPath(d.start_idx, 0)
	#print(f"Stop findPath: {path}")
	return path[1]