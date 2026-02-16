# Since we are building from source certain audio headers may not be present on Linux systems
# Check for ALSA or PulseAudio development files
if(UNIX AND NOT APPLE)
    find_package(PkgConfig REQUIRED)

    pkg_check_modules(ALSA alsa)
    pkg_check_modules(PULSE libpulse)

    if(NOT ALSA_FOUND AND NOT PULSE_FOUND)
        message(FATAL_ERROR
            "Neither ALSA nor PulseAudio development files were found.\n"
            "Please install one of the following:\n"
            "  sudo apt-get install libasound2-dev\n"
            "  sudo apt-get install libpulse-dev"
        )
    endif()
endif()

FetchContent_Declare(SDL2_mixer
    GIT_REPOSITORY https://github.com/libsdl-org/SDL_mixer.git
    GIT_TAG release-2.6.3
)
SET(SDL2MIXER_OPUS OFF)
SET(SDL2MIXER_FLAC OFF)
SET(SDL2MIXER_MOD OFF)
SET(SDL2MIXER_MIDI_FLUIDSYNTH OFF)
FetchContent_MakeAvailable(SDL2_mixer)