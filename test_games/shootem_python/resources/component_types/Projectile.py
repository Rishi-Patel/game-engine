import game_state


class Projectile:
    destroyed = False

    def OnStart(self):
        self.transform = self.actor.GetComponent("Transform")
        game_state.all_projectiles.append(self)

    def OnLateUpdate(self):
        if self.transform.x > 640:
            self.destroyed = True

        if self.destroyed:
            self.OnDestroy()
            Actor.Destroy(self.actor)

    # Manually called, not a lifecycle hook.
    def OnDestroy(self):
        if self in game_state.all_projectiles:
            game_state.all_projectiles.remove(self)
