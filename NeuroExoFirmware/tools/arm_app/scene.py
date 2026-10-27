"""Small software 3D renderer for Tk; no OpenGL or scientific stack required."""

import math


def cylinder(start, end, radius, color, segments=16):
    axis = [end[i] - start[i] for i in range(3)]
    length = math.sqrt(sum(v * v for v in axis))
    axis = [v / length for v in axis]
    helper = [0, 1, 0] if abs(axis[1]) < 0.9 else [1, 0, 0]
    u = [axis[1] * helper[2] - axis[2] * helper[1],
         axis[2] * helper[0] - axis[0] * helper[2],
         axis[0] * helper[1] - axis[1] * helper[0]]
    norm = math.sqrt(sum(v * v for v in u))
    u = [v / norm for v in u]
    v = [axis[1] * u[2] - axis[2] * u[1], axis[2] * u[0] - axis[0] * u[2], axis[0] * u[1] - axis[1] * u[0]]
    rings = [[tuple(point[j] + radius * (u[j] * math.cos(i * math.tau / segments) + v[j] * math.sin(i * math.tau / segments)) for j in range(3))
              for i in range(segments)] for point in (start, end)]
    faces = [(ring, color) for ring in rings]
    for i in range(segments):
        n = (i + 1) % segments
        faces.append(([rings[0][i], rings[0][n], rings[1][n], rings[1][i]], color))
    return faces


def arm_mesh(angle):
    radians = math.radians(angle)
    direction = (math.sin(radians), 0, -math.cos(radians))
    along = lambda t: tuple(t * value for value in direction)
    return (cylinder((0, 0, 0.06), (0, 0, 1.2), 0.105, "#8196ad")
            + cylinder((0, 0, 0.8), (0, 0, 1.02), 0.15, "#293f58")
            + cylinder(along(0.07), along(1.05), 0.10, "#49c8b7")
            + cylinder(along(0.64), along(0.85), 0.145, "#254e59")
            + cylinder((0, -0.15, 0), (0, 0.15, 0), 0.19, "#d2deeb")
            + cylinder((0, -0.17, 0), (0, -0.15, 0), 0.105, "#298f87")
            + cylinder(along(1.05), along(1.3), 0.075, "#a1b5c9"))


class ArmView:
    def __init__(self, canvas):
        self.canvas = canvas
        self.azimuth, self.elevation, self.zoom = 35, 18, 1.0
        self.angle, self.target = 0.0, None
        self.drag = None
        canvas.bind("<ButtonPress-1>", self._press)
        canvas.bind("<B1-Motion>", self._drag)
        canvas.bind("<MouseWheel>", lambda e: self._zoom(1.1 if e.delta > 0 else 0.9))
        canvas.bind("<Button-4>", lambda _: self._zoom(1.1))
        canvas.bind("<Button-5>", lambda _: self._zoom(0.9))
        canvas.bind("<Configure>", lambda _: self.draw())
        canvas.bind("<Double-Button-1>", lambda _: self.reset())

    def _press(self, event):
        self.drag = (event.x, event.y)

    def _drag(self, event):
        if self.drag:
            self.azimuth += (event.x - self.drag[0]) * 0.5
            self.elevation = max(-75, min(75, self.elevation + (event.y - self.drag[1]) * 0.5))
            self.drag = (event.x, event.y)
            self.draw()

    def _zoom(self, factor):
        self.zoom = max(0.5, min(2.5, self.zoom * factor))
        self.draw()

    def reset(self):
        self.azimuth, self.elevation, self.zoom = 35, 18, 1
        self.draw()

    def project(self, point):
        x, y, z = point
        az, el = math.radians(self.azimuth), math.radians(self.elevation)
        horizontal = x * math.cos(az) - y * math.sin(az)
        away = x * math.sin(az) + y * math.cos(az)
        vertical = z * math.cos(el) - away * math.sin(el)
        depth = away * math.cos(el) + z * math.sin(el)
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        scale = min(w, h) * 0.31 * self.zoom * 5 / (5 - depth)
        return (w / 2 + horizontal * scale, h / 2 - vertical * scale, depth)

    def draw(self, angle=None, target=None):
        if angle is not None:
            self.angle, self.target = angle, target
        canvas = self.canvas
        canvas.delete("all")
        for i in range(-4, 5):
            for a, b in (((i * 0.35, -1.4, -1.4), (i * 0.35, 1.4, -1.4)),
                         ((-1.4, i * 0.35, -1.4), (1.4, i * 0.35, -1.4))):
                p, q = self.project(a), self.project(b)
                canvas.create_line(*p[:2], *q[:2], fill="#1b3046")
        faces = []
        for points, color in arm_mesh(self.angle):
            projected = [self.project(point) for point in points]
            faces.append((sum(p[2] for p in projected) / len(projected), projected, color))
        for depth, points, color in sorted(faces, key=lambda face: face[0]):
            # Shade facets to show the cylindrical surfaces.
            shade = max(0.62, min(1.12, 0.86 + depth * 0.16))
            rgb = [min(255, int(int(color[i:i + 2], 16) * shade)) for i in (1, 3, 5)]
            fill = "#%02x%02x%02x" % tuple(rgb)
            canvas.create_polygon(*[v for p in points for v in p[:2]], fill=fill, outline=fill)
        if self.target is not None:
            r = math.radians(self.target)
            p = self.project((0, 0, 0))
            q = self.project((math.sin(r) * 1.3, 0, -math.cos(r) * 1.3))
            canvas.create_line(*p[:2], *q[:2], fill="#f5bb69", width=2, dash=(5, 4))
        canvas.create_text(16, 18, text="LIVE ARM  /  ENCODER ANGLE", fill="#8da5bc", anchor="nw", font=("Segoe UI", 9))
        canvas.create_text(16, canvas.winfo_height() - 22, text="Drag to orbit · Scroll to zoom · Double-click to reset", fill="#8da5bc", anchor="w", font=("Segoe UI", 9))
