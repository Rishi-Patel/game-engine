class PlayAudio:
    clip = ""
    loop = True

    def OnStart(self):
        Audio.Play(0, self.clip, True)
