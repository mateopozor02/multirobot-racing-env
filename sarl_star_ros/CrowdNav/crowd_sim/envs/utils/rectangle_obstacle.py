class RectangleObstacle:
    def __init__(self, center, width, height):
        self.center = center
        self.width = width
        self.height = height

    def contains(self, point):
        return (abs(point[0] - self.center[0]) < self.width / 2 and
                abs(point[1] - self.center[1]) < self.height / 2)

    def collides(self, other):
        return (abs(self.center[0] - other.center[0]) * 2 < (self.width + other.width) and
                abs(self.center[1] - other.center[1]) * 2 < (self.height + other.height))
    
    def get_vertices(self):
        x, y = self.center
        w, h = self.width / 2, self.height / 2
        return [(x - w, y - h), (x + w, y - h), (x + w, y + h), (x - w, y + h)]