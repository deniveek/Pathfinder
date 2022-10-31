import sys
import numpy as np
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from operator import xor
import RDP


class Boundary:
    def __init__(self, boundary):
        self.boundary = boundary

    @property
    def points(self):
        return self.boundary[0], self.boundary[1]


class Map:
    def __init__(self):
        self.robot_pos = None
        self.destination = None
        self.boundaries = []
        self.path = None
        self.trees = []

    def add_line(self, p1, p2):
        self.trees.append([p1, p2])

    def add_wall(self, wall):
        wall = RDP.DouglasPeucker(wall)
        res = np.stack([wall[:-1], wall[1:]], axis=1)
        for wall in res:
            self.boundaries.append(wall)
        return 0

    def add_robot(self, pos):
        self.robot_pos = pos
        return 0

    def add_destination(self, pos):
        self.destination = pos
        return 0

    @classmethod
    def intersects(cls, s0p0, s0p1, s1p0, s1p1):
        return xor(cls.is_right(s0p0, s0p1, s1p0), cls.is_right(s0p0, s0p1, s1p1)) and xor(cls.is_right(s1p0, s1p1, s0p0), cls.is_right(s1p0, s1p1, s0p1))

    @staticmethod
    def is_right(p0, p1, p):
        ab = p1 - p0
        ac = p - p0
        return True if np.cross(ac, ab) > 0 else False

    @classmethod
    def is_visible(cls, edges, p0, p1):
        #print(f"checking if visible for {p0}, {p1}")
        if len(edges) > 0:
            closing = next((s for s in edges if cls.intersects(p0, p1, s[0], s[1])), None)
            #print('closing')
            return closing is None
        return True


class Vertex:
    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent

    def __eq__(self, other):
        return np.array_equal(self.pos, other)


class Edge:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2


class Tree:
    def __init__(self, startpoint):
        self.vertices = [startpoint]
        self.edges = []

    def add_vertex(self, pos, parent=None):
        self.vertices.append(Vertex(pos, parent))

    def add_edge(self, p1, p2):
        self.edges.append(Edge(p1, p2))


class RRT:
    def __init__(self, start, goal, obstacles, vis, bounds=(800, 600), u=30, k=3000):
        self.u = u
        self.k = k
        self.bounds = bounds
        self.obstacles = obstacles
        self.vis = vis
        self.rng = np.random.default_rng(12345)
        self.trees = {}
        self.x_start = start
        self.x_goal = goal
        self.x_new = None
        #Tree(self.x_start)
        pass

    def build_RRT(self):
        print("started building rrt")
        self.trees['start'] = Tree(self.x_start)
        for i in range(self.k):
            x_rand = self.rng.integers([0, 0], [self.bounds], 2)
            print(f"iteration number {i}, sampling: {x_rand}")
            self.extend(self.trees['start'], x_rand)
        print('rrt built')
        return self.trees['start']

    def bidirRRT(self):
        self.vis.world.trees = []
        print("started bidirectional RRT")
        self.trees.update({'T_a': Tree(Vertex(self.x_start)), 'T_b': Tree(Vertex(self.x_goal))})
        active_tree = 'T_a'
        second_tree = 'T_b'
        for i in range(self.k):
            x_rand = self.rng.integers([0, 0], [self.bounds], 2)
            print(f"iteration number {i}, sampling: {x_rand}")
            if not self.extend(self.trees[active_tree], x_rand) == "Trapped":
                if self.extend(self.trees[second_tree], self.x_new) == "Reached":
                    return self.full_path(self.trees[active_tree], self.trees[second_tree])
            second_tree = active_tree
            active_tree = 'T_a' if active_tree == 'T_b' else 'T_b'
        print("finished bidirectional RRT, couldn't find a route tho")

    def extend(self, tree, x):
        x_near = self.nearest_neighbor(tree, x)
        try:
            self.x_new, u_new = self.new_state(x, x_near.pos)
            tree.add_vertex(self.x_new, x_near)
            self.vis.world.add_line(self.x_new, x_near.pos)
            self.vis.repaint()
            return "Reached" if np.array_equal(self.x_new, x) else "Advanced"
        except:
            return "Trapped"

    def new_state(self, x, x_near):
        x_new = np.int_(x_near + (x - x_near)/np.linalg.norm(x - x_near) * self.u) if self.u < np.linalg.norm(x - x_near) else x
        if Map.is_visible(self.obstacles, x_new, x_near):
            #print('visible')
            return x_new, self.u
        else:
            #print('obscured')
            return None

    @staticmethod
    def nearest_neighbor(tree, x):
        return tree.vertices[np.argmin(np.array([np.linalg.norm(x - v.pos) for v in tree.vertices]))]

    @staticmethod
    def path(T):
        path_a = [np.array([T.vertices[-1].pos, T.vertices[-1].parent.pos])]
        next_v = T.vertices[-1].parent
        while next_v.parent is not None:
            path_a.append(np.array([next_v.pos, next_v.parent.pos]))
            next_v = next_v.parent
        return path_a

    def full_path(self, T_a, T_b):
        path1 = self.path(T_a)
        path2 = self.path(T_b)
        return path1 + path2


class Window(QWidget):
    def __init__(self):
        super(Window, self).__init__()
        self.setWindowTitle("pathfinder")
        self.setFixedSize(QSize(800, 600))
        self.world = Map()
        self.state = 0
        self.line = []
        self.path = []
        #self.loadMap()
        self.show()

    @staticmethod
    def qtpoint2ndarray(point):
        return np.array([point.x(), point.y()])

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.state == 0:
                self.line = []
                self.line.append(event.pos())
                self.state = 1
        elif event.button() == Qt.RightButton:
            if self.world.robot_pos is None:
                pos = self.qtpoint2ndarray(event.pos())
                self.world.add_robot(pos)
            else:
                pos = self.qtpoint2ndarray(event.pos())
                self.world.add_destination(pos)
                pathfinder = RRT(self.world.robot_pos, self.world.destination, self.world.boundaries, self)
                self.world.path = pathfinder.bidirRRT()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.state == 1:
                self.world.add_wall([self.qtpoint2ndarray(p) for p in self.line])
                self.line = []
                self.state = 0
        self.update()

    def mouseMoveEvent(self, event):
        if self.state == 1:
            self.line.append(event.pos())
        self.update()

    def paintEvent(self, event):
        qp = QPainter()
        qp.begin(self)

        qp.setPen(QColor(Qt.red))
        qp.drawPolyline(QPolygon(self.line))
        for wall in self.world.boundaries:
            p1, p2 = wall[0], wall[1]
            qp.drawPolyline(QPoint(p1[0], p1[1]), QPoint(p2[0], p2[1]))
        qp.drawPolyline(QPolygon(self.line))

        if self.world.robot_pos is not None:
            qp.setPen(QColor(Qt.blue))
            qp.drawEllipse(QPoint(self.world.robot_pos[0], self.world.robot_pos[1]), 5, 5)
        if self.world.destination is not None:
            qp.setPen(QColor(Qt.magenta))
            qp.drawEllipse(QPoint(self.world.destination[0], self.world.destination[1]), 5, 5)
        if self.world.trees is not None:
            qp.setPen(Qt.gray)
            for edge in self.world.trees:
                qp.drawPolyline(QPoint(edge[0][0], edge[0][1]), QPoint(edge[1][0], edge[1][1]))
        if self.world.path is not None:
            qp.setPen(Qt.darkGreen)
            for edge in self.world.path:
                qp.drawPolyline(QPoint(edge[0][0], edge[0][1]), QPoint(edge[1][0], edge[1][1]))

        qp.end()


def main():
    app = QApplication(sys.argv)
    window = Window()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
