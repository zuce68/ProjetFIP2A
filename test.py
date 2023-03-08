import sounddevice as sd
import numpy as np
import threading
import wave

class Recorder:
    def __init__(self, filename):
        self.filename = filename
        self.frames = []
        self.recording = False

    def start_recording(self, channels, samplerate):
        self.recording = True
        self.frames = []
        threading.Thread(target=self.record, args=(channels, samplerate)).start()

    def stop_recording(self):
        self.recording = False

    def record(self, channels, samplerate):
        with wave.open(self.filename, mode='wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(2)
            wf.setframerate(samplerate)
            with sd.InputStream(channels=channels, samplerate=samplerate, blocksize=1024) as stream:
                while self.recording:
                    data, _ = stream.read(1024)
                    self.frames.append(data)
                    wf.writeframes(data)

    def get_frames(self):
        return self.frames

def play_sound():
    duration = 5.0
    samplerate = sd.query_devices('default')['default_samplerate']
    samples = int(duration * samplerate)
    sound = np.random.randn(samples, 1)
    sd.play(sound, samplerate=samplerate)
    sd.wait()

recorder = Recorder('enregistrement.wav')
recorder.start_recording(channels=1, samplerate=44100)

play_sound()

recorder.stop_recording()
frames = recorder.get_frames()

print('Enregistrement terminé.')
