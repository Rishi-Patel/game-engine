class DonnaAnimation:
    def OnStart(self):
        self.spriterenderer = self.actor.GetComponent("SpriteRenderer")

    def OnUpdate(self):
        anim_frame = Application.GetFrame() % 24
        if anim_frame < 6:
            self.spriterenderer.sprite = "donna1"
        elif anim_frame < 12:
            self.spriterenderer.sprite = "donna2"
        elif anim_frame < 18:
            self.spriterenderer.sprite = "donna1"
        else:
            self.spriterenderer.sprite = "donna3"
