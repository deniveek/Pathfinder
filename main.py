import sys
import numpy as np
import json
import RDP
import pathfinder
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


class Map():
	def __init__(self):
		self.walls = []

	def add_wall(self, wall: 'ndarray'):
		print(len(wall))
		wall_ = RDP.DouglasPeucker(wall)
		print(len(wall_))
		self.walls.append(wall_)


class Test(QWidget):
	def __init__(self):
		super(Test, self).__init__()
		self.setGeometry(100, 200, 600, 400)
		self.setWindowTitle("test")
		self.drawing = 0
		self.line = []
		self.path = []
		self.queue = False
		self.obstacles = Map()
		#self.loadMap()
		self.start_point = np.array([100, 100])
		self.fin_point = np.array([500, 300])
		self.show()

	def loadMap(self):
		try:
			with open("map.json", "r") as map:
				json_object = json.load(map)
				for wall in json_object:
					self.obstacles.add_wall(np.array(wall))
		except:
			print("no map file")
		# Update bbox
		#self.bbox = pathfinder.AugmentBBox(pathfinder.BBox(self.obstacles.walls), 1.1)
		return 0

	def findPath(self):
		#self.bbox = pathfinder.AugmentBBox(pathfinder.BBox(self.obstacles.walls), 1.1)
		self.path = pathfinder.findPath(self.obstacles.walls, self.start_point, self.fin_point)
		self.update()

	def qtpoint2ndarray(self, point):
		return np.array([point.x(), point.y()])

	def paintEvent(self, event):
		qp = QPainter()
		qp.begin(self)

		# Move coords to LeftTop of BBox
		#vp = pathfinder.AugmentBBox(self.bbox, 1.1)
		#qp.setWindow(vp[0], vp[1], vp[2] - vp[0], vp[3] - vp[1])

		# Draw obstacles
		qp.setPen(QColor(Qt.red))
		for wall in self.obstacles.walls:
			qp.drawPolyline(QPolygon([QPoint(p[0], p[1]) for p in wall]))
		qp.drawPolyline(QPolygon(self.line))

		# Draw start and stop points
		qp.setPen(QColor(Qt.blue))
		qp.drawEllipse(QPoint(self.start_point[0], self.start_point[1]), 5, 5)
		qp.drawEllipse(QPoint(self.fin_point[0], self.fin_point[1]), 5, 5)

		if len(self.path):
			qp.setPen(QColor(Qt.green))
			qp.drawPolyline(QPolygon([QPoint(p[0], p[1]) for p in self.path]))

		# Draw BBOX
		#qp.setPen(QColor(Qt.magenta))
		#qp.drawRect(self.bbox[0], self.bbox[1], self.bbox[2]-self.bbox[0], self.bbox[3]-self.bbox[1])

		qp.end()

	def mousePressEvent(self, event):
		if event.button() == Qt.RightButton:
			self.queue = event.pos()
		elif event.button() == Qt.LeftButton:
			if self.drawing == 1:
				self.line = []
				self.line.append(event.pos())
				self.drawing = 2
			elif np.linalg.norm(self.qtpoint2ndarray(event.pos()) - self.start_point) <= 5:
				self.drawing = 3
			elif np.linalg.norm(self.qtpoint2ndarray(event.pos()) - self.fin_point) <= 5:
				self.drawing = 4

	def mouseReleaseEvent(self, event):
		if event.button() == Qt.LeftButton:
			if self.drawing == 2:
				self.obstacles.add_wall(np.array([[p.x(), p.y()] for p in self.line]))
				self.line = []
				self.drawing = 1
			elif self.drawing == 3 or self.drawing == 4:
				self.drawing = 0

	def mouseMoveEvent(self, event):
		if self.drawing == 2:
			self.line.append(event.pos())
			self.update()
		elif self.drawing == 3:
			self.start_point = self.qtpoint2ndarray(event.pos())
		elif self.drawing == 4:
			self.fin_point = self.qtpoint2ndarray(event.pos())

	def keyPressEvent(self, event):
		if event.key() == Qt.Key_Alt and self.drawing == 0:
			self.drawing = 1
			#print('alt')
		elif event.key() == Qt.Key_S:
			to_file = [wall.tolist() for wall in self.obstacles.walls]
			file = json.dumps(to_file)
			with open("map.json", "w") as output:
				output.write(file)
			print('map saved')
		elif event.key() == Qt.Key_C:
			self.obstacles = Map()
			print('c event')
		elif event.key() == Qt.Key_L:
			self.loadMap()
		elif event.key() == Qt.Key_F:
			self.findPath()

	def keyReleaseEvent(self, event):
		if event.key() == Qt.Key_Alt:
			self.line = []
			self.drawing = 0
			#print('not alt')

	def closest_wall(self, point):
		dist_to_wall = 9999
		for wall in self.obstacles.walls:
			col = [Test.dist(point, p[0], p[1]) for p in list(zip(wall, wall[1:])) if np.cross(p[0], point) * np.cross(p[1], point) < 0]
			if len(col) and min(col) < dist_to_wall:
				dist_to_wall = min(col)
		return dist_to_wall

	@classmethod
	def dist(cls, point, start, end):
		if np.all(np.equal(start, end)):
			return np.linalg.norm(point - start)
		return np.divide(np.abs(np.linalg.norm(np.cross(start - end, start - point))), np.linalg.norm(end - start))


def main():
	app = QApplication(sys.argv)
	widg = Test()
	sys.exit(app.exec_())


if __name__ == '__main__':
	main()
