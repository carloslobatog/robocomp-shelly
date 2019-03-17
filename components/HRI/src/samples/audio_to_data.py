#!/usr/bin/env python3
# NOTE: this example requires PyAudio because it uses the Microphone class

import argparse
import uuid

# encoding=utf8  cuidado con las tiles
import sys
reload(sys)
sys.setdefaultencoding('utf8')


# credentianls for the grpc stub
import os
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] ="/home/lobato/Documentos/Shelly-58088bc904c2.json"
project_id = 'shelly-e0a42'
session_id = '000001'
audio_file_path = "/home/lobato/tfm/dialogflow-python-client-v2/samples/microphone-results.wav"
language_code = 'es-ES'

flag = False
while flag == False:
    # obtain audio from the microphone: to capture the audio chunks
    import speech_recognition as sr
    r = sr.Recognizer()
    with sr.Microphone(sample_rate = 16000) as source:
        print("Di algo!")
        audio = r.listen(source)

    # write audio to a WAV file
    # audiodata_instance.get_wav_data(convert_rate = None, convert_width = None)
    with open("microphone-results.wav", "wb") as f:
        f.write(audio.get_wav_data())

    # obtain the response from google DialogFlow
    import dialogflow_v2beta1 as dialogflow
    import detect_intent_with_texttospeech_response as tts

    session_client = dialogflow.SessionsClient()
    response = tts.detect_intent_with_texttospeech_response(project_id, session_id, audio_file_path, language_code)
    intent_detected = response.query_result.intent.display_name

    # play the output audio audio_file
    """
    import time, wave, pymedia.audio.sound as sound
    f= wave.open('output.wav', 'rb')
    sampleRate= f.getframerate()
    channels= f.getnchannels()
    format= sound.AFMT_S16_LE
    snd= sound.Output(sampleRate, channels, format)
    s= f.readframes(300000)
    snd.play(s)

    """

    # import pygame to play the resulted voice audio
    import pygame
    pygame.init()
    pygame.mixer.music.load("output.wav")
    pygame.mixer.music.play()

    # putting to sleep the system until the bot is stopped talking
    # calc the duration of the .wav
    import wave, contextlib, time
    fname = 'output.wav'
    with contextlib.closing(wave.open(fname,'r')) as f:
        frames = f.getnframes()
        rate = f.getframerate()
        duration = frames/float(rate)
    print(duration)
    time.sleep(duration + 0.5)

    # detect the intent of goodbye to exit the loop
    despedida = 'charla.despedida'
    if intent_detected == despedida:
        break

"""
# write audio to a FLAC file
with open("microphone-results.flac", "wb") as f:
    f.write(audio.get_flac_data())
"""
