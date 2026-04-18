#pragma once

#include <string>
#include <unordered_map>

#include "SDL_mixer.h"

class AudioManager
{
public:
    AudioManager(int numChannels);
    ~AudioManager();
    void AddAudio(const std::string& audioName);
    void PlayAudio(const std::string& audioName, int channel, int loops = -1);
    void StopAudio(int channel);
    void SetVolume(int channel, int volume);

private:

    std::unordered_map<std::string, Mix_Chunk*> _audio;
};