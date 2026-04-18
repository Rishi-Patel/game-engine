class ConstantMovement:
    x_vel = 0
    y_vel = 0

    def OnStart(self):
        self.transform = self.actor.GetComponent("Transform")

    def OnUpdate(self):
        self.transform.x = self.transform.x + self.x_vel
        self.transform.y = self.transform.y + self.y_vel
