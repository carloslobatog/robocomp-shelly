#
# Copyright (C) 2018 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time
import audio_player

from PySide import QtGui, QtCore
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
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
		audio_file_path = "/home/robocomp/robocomp/components/robocomp-araceli/components/HRI/microphone-results.wav"
		language_code = 'es-ES'


		situation = 0
		#one unique person blocking the route
		if situation == 1:
			context = "Aequam memento rebus" "in arduis servare mentem"
		#several people blocking the path
		if situation == 2:
			context = "Alohomora"

		if situation != 0:
			situation = 0
			# obtain the response from google DialogFlow
			import dialogflow_v2beta1 as dialogflow
			import text_to_texttospeech_response as ttts
			session_client = dialogflow.SessionsClient()

			response = ttts.text_to_texttospeech_response(project_id, session_id, context, language_code)
			intent_detected = response.query_result.intent.display_name
			audio_player.audio_player("output.wav")

		else: print('sigamos')


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
			import audio_to_texttospeech_response as atts

			session_client = dialogflow.SessionsClient()
			response = atts.audio_to_texttospeech_response(project_id, session_id, audio_file_path, language_code)
			intent_detected = response.query_result.intent.display_name

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
				time.sleep(10)
				break

		return True
