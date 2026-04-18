import math
import game_state


class Enemy:
    hp = 3
    damage_flash_cooldown = 5

    def OnStart(self):
        if game_state.game_over:
            return

        self.donna = Actor.Find("donna").GetComponent("Donna")
        self.donna_transform = Actor.Find("donna").GetComponent("Transform")
        self.transform = self.actor.GetComponent("Transform")
        self.spriterenderer = self.actor.GetComponent("SpriteRenderer")

        # Register this enemy in the global table.
        game_state.all_enemies.append(self)

    def OnUpdate(self):
        if game_state.game_over:
            return

        # Check for collision with player.
        player_pos = (self.donna_transform.x, self.donna_transform.y)
        our_pos = (self.transform.x, self.transform.y)

        distance = Enemy._distance(our_pos, player_pos)
        if distance < 50:
            self.donna.TakeDamage()

        # check for collision with projectile.
        for projectile in game_state.all_projectiles:
            projectile_transform = projectile.actor.GetComponent("Transform")
            projectile_position = (projectile_transform.x, projectile_transform.y)
            distance = Enemy._distance(our_pos, projectile_position)
            if distance <= 50:
                self.TakeDamage()
                projectile.destroyed = True

        # Damage Flash
        if self.damage_flash_cooldown > 0:
            self.damage_flash_cooldown -= 1
            if Application.GetFrame() % 4 > 1:
                self.spriterenderer.g = 0
                self.spriterenderer.b = 0
            else:
                self.spriterenderer.g = 255
                self.spriterenderer.b = 255
        else:
            self.spriterenderer.g = 255
            self.spriterenderer.b = 255

    # Manually called by other components, not a lifecycle hook.
    def OnDestroy(self):
        if self in game_state.all_enemies:
            game_state.all_enemies.remove(self)

    @staticmethod
    def _distance(p1, p2):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return math.sqrt(dx * dx + dy * dy)

    def TakeDamage(self):
        self.damage_flash_cooldown = 5
        self.hp -= 1

        if self.hp <= 0:
            game_state.score += 1
            Actor.Destroy(self.actor)
