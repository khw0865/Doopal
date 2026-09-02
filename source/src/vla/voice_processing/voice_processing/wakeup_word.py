import os
import numpy as np
from openwakeword.model import Model
from scipy.signal import resample_poly
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

MODEL_NAME = "hey_doopal_final.tflite"
MODEL_PATH = os.path.join(PACKAGE_PATH, f"resource/{MODEL_NAME}")

class WakeupWord:
    def __init__(self, buffer_size):
        self.model = None
        self.model_name = MODEL_NAME.split(".", maxsplit=1)[0]
        self.stream = None
        self.buffer_size = buffer_size

    def is_wakeup(self):
        audio_chunk = np.frombuffer(
            self.stream.read(self.buffer_size, exception_on_overflow=False),
            dtype=np.int16,
        )
        # audio_chunk = resample_poly(audio_chunk, int(len(audio_chunk) * 16000 / 48000))
        audio_chunk = resample_poly(
            audio_chunk,
            up=1,
            down=3,
        ).astype(np.int16)
        outputs = self.model.predict(audio_chunk, threshold=0.1)
        confidence = outputs[self.model_name]
        print("confidence: ", confidence)
        # Wakeword 탐지
        if confidence > 0.5:
            print("Wakeword detected!")
            return True
        return False

    def set_stream(self, stream):
        self.model = Model(
            wakeword_models=[MODEL_PATH],
            inference_framework="tflite",
        )

        self.model_name = next(
            iter(self.model.models.keys())
        )

        required_frames = int(
            self.model.model_inputs[self.model_name]
        )

        while (
            self.model.preprocessor.feature_buffer.shape[0]
            < required_frames
        ):
            self.model.preprocessor(
                np.zeros(1280, dtype=np.int16)
            )

        print(
            "모델:",
            self.model_name,
            "버퍼:",
            self.model.preprocessor.feature_buffer.shape[0],
            "/",
            required_frames,
        )

        self.stream = stream

    # def set_stream(self, stream):
    #     self.model = Model(wakeword_models=[MODEL_PATH])
    #     self.stream = stream