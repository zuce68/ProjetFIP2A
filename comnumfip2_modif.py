"""
TRAITESIGNFIP - Module Python pour les communications numériques à Télécom Physique Strasbourg (spécifiquement pour la formation FIP EII).

Copyright Université de Strasbourg 2023 (2023-03-15)
Contributeur : pierre.misiuk@etu.unistra.fr
               mathieu.schwoerer@etu.unistra.fr
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io.wavfile import write
import sounddevice as sd

def send_signal_to_canal(signal):
    sd.play(signal, 44100)
    sd.wait()
    
def plot_signal(signal):
    plt.figure()
    plt.plot(signal)
    plt.show()

def AMI_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 10000  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    # Générer le signal AMI
    temps_bit = np.linspace(0, duree_bit, int(duree_bit * framerate), endpoint=False)
    signal_ami = np.zeros(0)
    precedent_bit = 0
    for bit in bits:
        if bit == 0:
            signal_ami = np.append(signal_ami, amplitude * np.ones_like(temps_bit))
        else:
            signal_ami = np.append(signal_ami, -amplitude * np.ones_like(temps_bit))
            precedent_bit = bit
             # Ajouter des zéros au début et à la fin du signal
    signal_ami = np.pad(signal_ami, (int(framerate/2), int(framerate/2)), 'constant')
    return signal_ami

def NRZ_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 1   # Amplitude du signal
    frequence = 4410  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    # Générer le signal NRZ
    temps_bit = np.linspace(0, duree_bit, int(duree_bit * framerate), endpoint=False)
    signal_nrz = np.zeros(0)
    for bit in bits:
        if bit == 0:
            signal_nrz = np.append(signal_nrz, amplitude * np.ones_like(temps_bit))
        else:
            signal_nrz = np.append(signal_nrz, -amplitude * np.ones_like(temps_bit))
         # Ajouter des zéros au début et à la fin du signal
    signal_nrz = np.pad(signal_nrz, (int(framerate/2), int(framerate/2)), 'constant')
    print(bits)
    print(len(bits))
    return signal_nrz

def manchester_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 10000  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    # Générer le signal Manchester
    temps_bit = np.linspace(0, duree_bit, int(duree_bit * framerate), endpoint=False)
    signal_manchester = np.zeros(0)
    for bit in bits:
        if bit == 0:
            signal_manchester = np.append(signal_manchester, amplitude * np.ones_like(temps_bit))
            signal_manchester = np.append(signal_manchester, -amplitude * np.ones_like(temps_bit))
        else:
            signal_manchester = np.append(signal_manchester, -amplitude * np.ones_like(temps_bit))
            signal_manchester = np.append(signal_manchester, amplitude * np.ones_like(temps_bit))
     # Ajouter des zéros au début et à la fin du signal
    signal_manchester = np.pad(signal_manchester, (int(framerate/2), int(framerate/2)), 'constant')

    return signal_manchester


