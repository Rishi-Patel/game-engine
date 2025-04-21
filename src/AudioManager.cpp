#include "AudioManager.h"

#include "AudioHelper.h"

#include <array>

#include <stdexcept>

constexpr auto SOUND_FREQUENCY = 22050;
constexpr auto NUM_CHANNELS = 2; //Stereo vs mono
constexpr auto SAMPLE_SIZE = 4096;
constexpr auto BGM_CHANNEL = 0;


static constexpr std::array<char[5], 2> ALLOWED_AUDIO_EXT = { ".wav", ".ogg" };
static const std::filesystem::path AUDIO_CONFIG_DIR = std::filesystem::path("resources") / std::filesystem::path("audio");

std::filesystem::path GetAudioFilePath(const std::string& audioFile) {
    std::filesystem::path audioPath;
    for (const auto& ext : ALLOWED_AUDIO_EXT)
    {
        audioPath = AUDIO_CONFIG_DIR / (audioFile + ext);
        if (std::filesystem::exists(audioPath))
        {
            return audioPath;
        }
    }
    std::cout << "error: failed to play audio clip " << audioFile;
    exit(0);
}

AudioManager::AudioManager(int numChannels)
{
    if (AudioHelper::Mix_OpenAudio(SOUND_FREQUENCY, MIX_DEFAULT_FORMAT, NUM_CHANNELS, SAMPLE_SIZE) == -1)
    {
        throw std::runtime_error("SDL Mixer could not initialize! SDL_Error: " + std::string(SDL_GetError()));
    }
    AudioHelper::Mix_AllocateChannels(numChannels);
}

void AudioManager::AddAudio(const std::string& audioName)
{
    auto audioPath = GetAudioFilePath(audioName);
    _audio.emplace(audioName, AudioHelper::Mix_LoadWAV(audioPath.string().c_str()));
}

void AudioManager::PlayAudio(const std::string& audioName, int channel, int loops)
{
    if (_audio.find(audioName) == _audio.end()) {
        AddAudio(audioName);
    }
    AudioHelper::Mix_PlayChannel(channel, _audio.at(audioName), loops);
}

void AudioManager::StopAudio(int channel)
{
    AudioHelper::Mix_HaltChannel(channel);
}

void AudioManager::SetVolume(int channel, int volume) {
    AudioHelper::Mix_Volume(channel, std::clamp(volume, 0, 128));
}

AudioManager::~AudioManager()
{
    AudioHelper::Mix_CloseAudio();
}