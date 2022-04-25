

class Obstacle:

    def __init__(self, clearance):
        self.x = 10 # [m] Width in meters
        self.y = 10 # [m] Height in meters
        self.clearance = clearance
        self.robot_radius = 0.354 / 2
        self.clearance = self.robot_radius + self.clearance

        self.circle1_radius = 0.5
        self.circle2_radius = 1
        self.circle1_x_offset = 2
        self.circle1_y_offset = 2
        self.circle2_x_offset = 2
        self.circle2_y_offset = 8

        self.square1_corner1_x = 0.25
        self.square1_corner1_y = 4.25
        self.square_side = 1.5

        self.rect1_corner1_x = 3.75
        self.rect1_corner1_y = 4.25
        self.rect1_length = 2.5
        self.rect1_width = 1.5

        self.rect2_corner1_x = 7.25
        self.rect2_corner1_y = 2
        self.rect2_length = 1.5
        self.rect2_width = 2

    def in_collision(self, x, y):
        if (x < 0 or x >= 10 or y < 0 or y >= 10):
          return 1

        x_offset = self.circle1_x_offset
        y_offset = self.circle1_y_offset
        radius = self.circle1_radius + self.clearance
        if ((x-x_offset)**2 + (y-y_offset)**2 <= radius):
          return 1

        x_offset = self.circle2_x_offset
        y_offset = self.circle2_y_offset
        radius = self.circle2_radius + self.clearance
        if ((x-x_offset)**2 + (y-y_offset)**2 <= radius):
          return 1

        x1 = self.square1_corner1_x - self.clearance
        x2 = x1 + self.square_side + 2*self.clearance
        y1 = self.square1_corner1_y - self.clearance
        y2 = y1 + self.square_side  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            return 1

        x1 = self.rect1_corner1_x - self.clearance
        x2 = x1 + self.rect1_length + 2*self.clearance
        y1 = self.rect1_corner1_y - self.clearance
        y2 = y1 + self.rect1_width  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            return 1

        x1 = self.rect2_corner1_x - self.clearance
        x2 = x1 + self.rect2_length + 2*self.clearance
        y1 = self.rect2_corner1_y - self.clearance
        y2 = y1 + self.rect2_width  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            return 1

        return 0