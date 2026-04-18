class RectangleRenderer:
    x = 430
    y = 240
    width = 100
    height = 60
    r = 220.0
    g = 60.0
    b = 60.0
    a = 255.0

    def OnUpdate(self):
        right = self.x + self.width
        bottom = self.y + self.height
        for py in range(self.y, bottom):
            for px in range(self.x, right):
                Image.DrawPixel(px, py, self.r, self.g, self.b, self.a)
