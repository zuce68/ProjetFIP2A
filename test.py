import sounddevice as sd
import soundfile as sf
import numpy as np

# Paramètres de l'enregistrement audio
duration = 5  # durée de l'enregistrement en secondes
fs = 44100  # fréquence d'échantillonnage en Hz
channels = 1  # nombre de canaux audio (mono = 1)

# Fréquence du son émis
f = 440

# Génération du signal sonore
t = np.linspace(0, duration, int(duration * fs), False)
sinewave = np.sin(f * 2 * np.pi * t)

# Émission du signal sonore
sd.play(sinewave, fs)

# Enregistrement de l'acquisition audio dans un fichier
print("Enregistrement audio en cours...")
recording = sd.rec(int(duration * fs), samplerate=fs, channels=channels)
sd.wait()
sf.write("enregistrement.wav", recording, fs)
print("Enregistrement audio terminé.")
