#!/usr/bin/env python

# Copyright 2018 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Dialogflow API Beta Detect Intent Python sample with an audio response.

Examples:
  python detect_intent_with_texttospeech_response.py -h
  python detect_intent_with_texttospeech_response.py --project-id PROJECT_ID \
  --session-id SESSION_ID "hello"
"""

import argparse
import uuid


# [START dialogflow_detect_intent_with_texttospeech_response]
def detect_intent_with_texttospeech_response(project_id,
                                             session_id,
                                             audio_file_path,
                                             language_code
                                             ):
    """Returns the result of detect intent with texts as inputs and includes
    the response in an audio format.

    Using the same `session_id` between requests allows continuation
    of the conversaion."""
    import dialogflow_v2beta1 as dialogflow
    session_client = dialogflow.SessionsClient()

    # Note: hard coding audio_encoding and sample_rate_hertz for simplicity.
    audio_encoding = dialogflow.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16
    sample_rate_hertz = 16000

    session = session_client.session_path(project_id, session_id)
    print('Session path: {}\n'.format(session))

    with open(audio_file_path, 'rb') as audio_file:
        input_audio = audio_file.read()

    audio_config = dialogflow.types.InputAudioConfig(
        audio_encoding=audio_encoding, language_code=language_code,
        sample_rate_hertz=sample_rate_hertz)
    query_input = dialogflow.types.QueryInput(audio_config=audio_config)

    # Enable sentiment analysis
    sentiment_config = dialogflow.types.SentimentAnalysisRequestConfig(
        analyze_query_text_sentiment=True)

    # Set the query parameters with sentiment analysis
    query_params = dialogflow.types.QueryParameters(
        sentiment_analysis_request_config=sentiment_config)

    # Set the output audio config
    output_audio_config = dialogflow.types.OutputAudioConfig(
        audio_encoding=dialogflow.enums.OutputAudioEncoding
        .OUTPUT_AUDIO_ENCODING_LINEAR_16)

    response = session_client.detect_intent(
        session=session,
        query_input=query_input,
        input_audio=input_audio,
        query_params=query_params,
        output_audio_config=output_audio_config)

    print('=' * 20)
    print('Query text: {}'.format(response.query_result.query_text))
    print('Detected intent: {} (confidence: {})\n'.format(
        response.query_result.intent.display_name,
        response.query_result.intent_detection_confidence))
    print('Fulfillment text: {}\n'.format(
        response.query_result.fulfillment_text))

    # Score between -1.0 (negative sentiment) and 1.0 (positive sentiment).
    print('Query Text Sentiment Score: {}\n'.format(
        response.query_result.sentiment_analysis_result
        .query_text_sentiment.score))
    print('Query Text Sentiment Magnitude: {}\n'.format(
        response.query_result.sentiment_analysis_result
        .query_text_sentiment.magnitude))

    # The response's audio_content is binary.
    with open('output.wav', 'wb') as out:
        out.write(response.output_audio)
        print('Audio content written to file "output.wav"')

    return response
# [END dialogflow_detect_intent_with_texttospeech_response]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--project-id',
        help='Project/agent id.  Required.',
        required=True)
    parser.add_argument(
        '--session-id',
        help='Identifier of the DetectIntent session. '
             'Defaults to a random UUID.',
        default=str(uuid.uuid4()))
    parser.add_argument(
        '--language-code',
        help='Language code of the query. Defaults to "en-US".',
        default='en-US')
    parser.add_argument(
        '--audio-file-path',
        help='Path to the audio file.',
        required=True)

    args = parser.parse_args()

    detect_intent_with_texttospeech_response(
        args.project_id, args.session_id, args.audio_file_path, args.language_code)
