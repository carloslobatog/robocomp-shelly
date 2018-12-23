import pygame
pygame.init()

def audio_player(input_audio):
    # import pygame to play the resulted voice audio
    pygame.mixer.music.load(input_audio)
    pygame.mixer.music.play()
    # putting to sleep the system until the bot is stopped talking
    # calc the duration of the .wav
    import wave, contextlib, time
    fname = input_audio
    with contextlib.closing(wave.open(fname,'r')) as f:
        frames = f.getnframes()
        rate = f.getframerate()
        duration = frames/float(rate)
    print(duration)
    time.sleep(duration + 0.5)
