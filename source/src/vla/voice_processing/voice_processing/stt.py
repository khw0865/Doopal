import numpy as np
import scipy.io.wavfile as wav
import tempfile
from openai import OpenAI

class STT:
    def __init__(self, openai_api_key):
        self.client = OpenAI(api_key=openai_api_key)
        self.samplerate = 48000 # MicController 설정에 맞춰야 함

    def speech2text(self, stream):
        print("음성 녹음을 시작합니다. (5초 동안 마이크 데이터 수집)")
        
        frames = []
        # 3초 동안 기존 stream에서 데이터 읽기 (CHUNK_SIZE는 3840으로 가정)
        # 48000Hz * 3초 / 3840 = 37.5회 반복
        for _ in range(63): 
            data = stream.read(3840, exception_on_overflow=False)
            frames.append(data)
        
        print("녹음 완료. Whisper에 전송 중...")

        audio_data = b''.join(frames)
        audio_np = np.frombuffer(audio_data, dtype=np.int16)

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            wav.write(temp_wav.name, self.samplerate, audio_np)

            with open(temp_wav.name, "rb") as f:
                transcript = self.client.audio.transcriptions.create(
                    model="whisper-1", file=f)

        print("STT 결과: ", transcript.text)
        return transcript.text