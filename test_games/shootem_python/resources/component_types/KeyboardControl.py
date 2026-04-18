class KeyboardControl:
    speed = 3

    def OnStart(self):
        self.transform = self.actor.GetComponent("Transform")

    def OnUpdate(self):
        vel_x = 0
        vel_y = 0

        if Input.GetKey("right"):
            vel_x += self.speed
        if Input.GetKey("left"):
            vel_x -= self.speed
        if Input.GetKey("up"):
            vel_y -= self.speed
        if Input.GetKey("down"):
            vel_y += self.speed

        self.transform.x = self.transform.x + vel_x
        self.transform.y = self.transform.y + vel_y

        if self.transform.x > 600:
            self.transform.x = 600
        if self.transform.x < 0:
            self.transform.x = 0
        if self.transform.y > 320:
            self.transform.y = 320
        if self.transform.y < 0:
            self.transform.y = 0
