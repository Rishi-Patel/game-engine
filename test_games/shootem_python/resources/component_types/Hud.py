import game_state


class Hud:
    transition_y = -360
    transition_y_vel = 0
    transition_frames = 0
    played_ending_audio_clip = False

    def OnStart(self):
        self.donna = Actor.Find("donna").GetComponent("Donna")
        Audio.Play(1, "mission_start_vocal", False)

    def OnLateUpdate(self):
        # Draw Score
        Text.Draw("Score : " + str(game_state.score), 500, 10,
                  "NotoSans-Regular", 20, 255, 255, 255, 255)

        # Draw lives
        Image.DrawUI("hud", 0, 0)
        lives = self.donna.lives

        for i in range(lives):
            x_pos = 15
            y_pos = 20
            Image.DrawUI("life", x_pos + 50 * i, y_pos - 8 * i)

        # Draw Game Over
        if lives <= 0:
            game_state.game_over = True

            # Audio clips
            if self.transition_frames == 0:
                Audio.Play(0, "ThisGameIsOver_by_mccartneytm_ccby", False)

            self.transition_frames += 1
            if self.transition_frames > 150 and not self.played_ending_audio_clip:
                Audio.Play(1, "game_over_vocal", False)
                self.played_ending_audio_clip = True

            # Accelerate downwards
            self.transition_y_vel += 0.2
            self.transition_y += self.transition_y_vel

            # bounce
            if self.transition_y > 0:
                self.transition_y = 0
                self.transition_y_vel = -self.transition_y_vel * 0.3

            Image.DrawUIEx("game_over_bad", 0, self.transition_y,
                           255, 255, 255, 255, 2)
