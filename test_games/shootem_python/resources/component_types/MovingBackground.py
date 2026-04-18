class MovingBackground:
    def OnStart(self):
        self.transform = self.actor.GetComponent("Transform")

    def OnUpdate(self):
        self.transform.x = self.transform.x - 1
        if self.transform.x <= -667:
            self.transform.x = 667
