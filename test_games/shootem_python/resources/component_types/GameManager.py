import random
import game_state


class GameManager:
    num_enemies = 10
    base_enemy_movement_speed = -1.5

    def OnStart(self):
        random.seed(494)

        # Establish necessary references.
        self.donna = Actor.Find("donna").GetComponent("Donna")

        # Reset global collections.
        game_state.all_enemies = []
        game_state.all_projectiles = []
        game_state.score = 0
        game_state.game_over = False

    def OnUpdate(self):
        self.base_enemy_movement_speed -= 0.001

        if Application.GetFrame() % 60 == 0 and not game_state.game_over:
            self.SpawnEnemy()

    def SpawnEnemy(self):
        new_enemy = Actor.Instantiate("Enemy")
        enemy_transform = new_enemy.GetComponent("Transform")

        # Determine random sprite.
        possible_sprites = ["enemy1", "enemy2", "enemy3"]
        chosen_sprite = random.choice(possible_sprites)
        new_enemy.GetComponent("SpriteRenderer").sprite = chosen_sprite
        new_enemy.GetComponent("ConstantMovement").x_vel = (
            self.base_enemy_movement_speed + random.random() * -1.0
        )

        # Determine random position.
        enemy_transform.x = 700 + random.random() * 200
        enemy_transform.y = 50 + random.random() * 250
