# Final Demo for A* Algorithm
# python3 ~/Documents/NCKH/demo_mesh/final_grid_input_color.py
# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
import numpy as np
import heapq

# ===============================
# LOAD INPUT (chỉ obstacle)
# ===============================
def load_input(filename):
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f if line.strip() and not line.startswith('#')]

    w, h = map(int, lines[0].split())
    sx, sy = map(int, lines[1].split())
    gx, gy = map(int, lines[2].split())

    grid = []
    for i in range(3, 3 + h):
        grid.append(list(map(int, lines[i].split())))

    return w, h, (sx, sy), (gx, gy), np.array(grid)

# ===============================
# A* 2.5D
# ===============================
def astar_25d(obs, height, start, goal):
    h, w = obs.shape
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}

    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    while open_set:
        _, cur = heapq.heappop(open_set)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = cur[0]+dx, cur[1]+dy

            if 0 <= nx < h and 0 <= ny < w:
                if obs[nx, ny] == 1:
                    continue

                base = 1
                h_cost = height[nx, ny] * 5
                slope = abs(height[nx, ny] - height[cur]) * 10

                cost = base + h_cost + slope
                g_new = g_score[cur] + cost

                if (nx, ny) not in g_score or g_new < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = g_new
                    f = g_new + heuristic((nx, ny), goal)
                    heapq.heappush(open_set, (f, (nx, ny)))
                    came_from[(nx, ny)] = cur

    return None

# ===============================
# NODE
# ===============================
class DemoNode(Node):
    def __init__(self):
        super().__init__('final_grid_input_color')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.grid_pub = self.create_publisher(Marker, 'grid', 10)
        self.path_pub = self.create_publisher(Marker, 'path', 10)

        self.timer = self.create_timer(1.0, self.publish)

        # ===== LOAD FILE =====
        file_path = "/home/lam/Documents/NCKH/demo_mesh/input.txt"
        self.w, self.h, self.start, self.goal, self.obs = load_input(file_path)

        # ===== HEIGHT (auto) =====
        self.height = np.zeros((self.w, self.h))
        for i in range(self.w):
            for j in range(self.h):
                self.height = np.random.rand(self.w, self.h)

        self.height[self.obs == 1] += 1.2

        self.obs[self.start] = 0
        self.obs[self.goal] = 0

        # ===== PATH =====
        self.path = astar_25d(self.obs, self.height, self.start, self.goal)

        if self.path:
            self.get_logger().info("✅ Path found")
        else:
            self.get_logger().warn("❌ No path")

    # ===============================
    # TF
    # ===============================
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    # ===============================
    # COLOR (6 levels)
    # ===============================
    def height_to_color(self, h):
        c = ColorRGBA()
        c.a = 1.0

        h = float(min(max(h, 0.0), 1.0))

        palette = [
            (0.0,1.0,0.0),   # xanh lá
            (0.0,0.6,0.0),   # xanh lá đậm
            (1.0,1.0,0.0),   # vàng
            (1.0,0.5,0.0),   # cam
            (1.0,0.0,0.0),   # đỏ
            (0.5,0.0,0.0)    # đỏ đậm
        ]

        idx = min(int(h * len(palette)), len(palette)-1)

        r,g,b = palette[idx]
        c.r = float(r)
        c.g = float(g)
        c.b = float(b)

        return c

    # ===============================
    # GRID
    # ===============================
    def publish_grid(self):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.CUBE_LIST

        m.scale.x = 0.9
        m.scale.y = 0.9
        m.scale.z = 0.01

        for i in range(self.w):
            for j in range(self.h):
                p = Point()
                p.x = j - self.w/2
                p.y = i - self.h/2
                p.z = 0.0

                m.points.append(p)

                if self.obs[i,j] == 1:
                    c = ColorRGBA()
                    c.r = c.g = c.b = 0.0
                    c.a = 1.0
                else:
                    c = self.height_to_color(self.height[i,j])

                m.colors.append(c)

        self.grid_pub.publish(m)

    # ===============================
    # PATH
    # ===============================
    def publish_path(self):
        if not self.path:
            return

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.15

        m.color.r = 0.6
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0

        for (x,y) in self.path:
            p = Point()
            p.x = y - self.w/2
            p.y = x - self.h/2
            p.z = 0.1
            m.points.append(p)

        self.path_pub.publish(m)

    # ===============================
    def publish(self):
        self.publish_tf()
        self.publish_grid()
        self.publish_path()

# ===============================
def main():
    rclpy.init()
    node = DemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()