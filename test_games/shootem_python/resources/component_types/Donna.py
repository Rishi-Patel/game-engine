import math
import game_state


class Donna:
    lives = 3
    iframe_cooldown = 0

    def OnStart(self):
        self.spriterenderer = self.actor.GetComponent("SpriteRenderer")
        self.transform = self.actor.GetComponent("Transform")

    def OnUpdate(self):
        # projectile spawning
        if Input.GetKey("space") and Application.GetFrame() % 10 == 0:
            new_projectile = Actor.Instantiate("Projectile")
            new_projectile_transform = new_projectile.GetComponent("Transform")
            new_projectile_transform.x = self.transform.x
            new_projectile_transform.y = self.transform.y

        # iframes
        if self.iframe_cooldown > 0:
            self.iframe_cooldown -= 1

            # Damage flash red
            sin_value_01 = (math.sin(Application.GetFrame() * 0.4) + 1) * 0.5
            self.spriterenderer.g = 255 * sin_value_01
            self.spriterenderer.b = 255 * sin_value_01
            self.spriterenderer.r = 255
        else:
            # Become normal color again
            self.spriterenderer.g = 255
            self.spriterenderer.b = 255
            self.spriterenderer.r = 255

    def TakeDamage(self):
        if self.iframe_cooldown <= 0:
            if not game_state.game_over:
                Audio.Play(1, "ouch", False)
            self.lives -= 1
            self.iframe_cooldown = 90

        if self.lives <= 0:
            Actor.Destroy(self.actor)
