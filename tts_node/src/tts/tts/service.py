import rclpy
import os
import subprocess
from std_msgs.msg import String
from google.oauth2 import service_account
from google.cloud import texttospeech
from tts_interfaces.srv import Interfaces

def tts_callback(request, response):
    text = request.text

    try:
        client = texttospeech.TextToSpeechClient(credentials=creds)
        synthesis_input = texttospeech.SynthesisInput(text=text)
        voice = texttospeech.VoiceSelectionParams(
            language_code="ko-KR", name="ko-KR-Wavenet-A"
        )
        audio_config = texttospeech.AudioConfig(audio_encoding=texttospeech.AudioEncoding.LINEAR16)

        synthesis_response = client.synthesize_speech(input=synthesis_input, voice=voice, audio_config=audio_config)

        audio_data = synthesis_response.audio_content

        # 저장한 임시 오디오 파일을 생성
        audio_file_path = "/tmp/audio.wav"
        with open(audio_file_path, "wb") as audio_file:
            audio_file.write(audio_data)

        # aplay를 사용하여 오디오 재생
        subprocess.run(["aplay", "-D", "plughw:1,0", audio_file_path])

        # 성공 응답 반환
        response.result = 'success'

        # 임시 오디오 파일 삭제
        os.remove(audio_file_path)

    except Exception as e:
        response.result = 'fail'
        print(f"Error: {str(e)}")

    return response

def main():
    global creds
    creds = service_account.Credentials.from_service_account_file('/root/key.json')

    rclpy.init()
    node = rclpy.create_node("tts_node")
    service = node.create_service(Interfaces, "/text_to_speech", tts_callback)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
