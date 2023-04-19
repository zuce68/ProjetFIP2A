"""
TRAITESIGNFIP - Module Python pour les communications numériques à Télécom Physique Strasbourg (spécifiquement pour la formation FIP EII).

Copyright Université de Strasbourg 2023 (2023-03-15)
Contributeur : pierre.misiuk@etu.unistra.fr
               mathieu.schwoerer@etu.unistra.fr
"""

import sounddevice as sd
import soundfile as sf
# Bibliothèques pour analyse traitement du signal
import scipy.io.wavfile
import matplotlib.pyplot as plt
import numpy as np
import threading

def read_signal(file):
    """
    Permet de créer un tableau avec les valeurs d'un fichier son. 
    
    Entrées :
    file (string)      : nom du fichier audio.
    
    Sortie :
    data_left (tableau) : Données du premier canal.
    data_right (tableau) : Données du deuxième canal.
    """
    rate, data = scipy.io.wavfile.read('enregistrement.wav')#Lecture du fichier où l'acquisition a été enregistrée 
    data_right = data[:,1]
    data_left = data[:,0]
    return data_left,data_right

def play_signal(signal, sample_rate):
    """
    Jouer un son sur l'haut-parleur.
    
    Entrées :
    signal (array)      : signal
    sample_rate (int)      : fréquence d'échantillonnage
    
    Sortie :
    aucune
    """
    sd.play(signal, sample_rate)
    sd.wait()

# Fonction pour enregistrer le son du microphone
def record_microphone(signal_type,time):
    """
    Acquisition du son via microphone.
    
    Entrées :
    signal_type (string)      : nom du signal envoyé sr l'haut-parleur pour l'acquérir avec les micros.
    time (decimal) : valeur de l'enregistrement en seconde 

    signal_type:
    -noise : bruit blanc.
    -sinus : sinusoïde de 440Hz.
    -clap : simulation d'un claquement avec une sinusoïde.
    
    Sortie :
    aucune
    """
    global samples
    if time==0:
        duration=5
    else:
        duration = int((5*time)/3)  # Durée de l'enregistrement en secondes
    sample_rate = 44100  # Fréquence d'échantillonnage en Hz
    channels = 2

    t = np.linspace(0,1*time,int(sample_rate),endpoint=False)
 
    
    if signal_type == "noise":
        zeros = np.zeros((duration-time)*sample_rate)
        samples = np.random.normal(0, 1, time*sample_rate)
        signal = np.concatenate((zeros,samples))
    elif signal_type == "sinus":
        zeros = np.zeros((duration-time)*sample_rate)
        frequency = 440  # Hz
        samples = (np.sin(2 * np.pi * np.arange(sample_rate * time ) * frequency / sample_rate)).astype(np.float32)
        signal = np.concatenate((zeros,samples))
    elif signal_type == "clap":
        zeros = np.zeros((duration-time)*sample_rate)
        samples=np.sin(2*np.pi*500*t)*np.exp(-5*t)
        signal = np.concatenate((zeros,samples))
    elif signal_type == "edge":
        zeros = np.zeros((duration-time)*sample_rate)
        frequency = 440  # Hz
        samples = np.sign((np.sin(2 * np.pi * np.arange(sample_rate * time ) * frequency / sample_rate))).astype(np.float32)
        signal = np.concatenate((zeros,samples))
    else:
        print("Type de signal non reconnu.")
        return

    # Démarrer un thread pour lire le signal sur les haut-parleurs
    play_thread = threading.Thread(target=play_signal, args=(signal, sample_rate))
    play_thread.start()

    # Enregistrer le son du microphone dans un fichier WAV
    recorded_audio = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=channels)
    sd.wait()  # Attendre la fin de l'enregistrement
    # Enregistrer les données audio dans un fichier WAV
    file_name = "enregistrement.wav"
    sf.write(file_name, recorded_audio, sample_rate, subtype='PCM_16')
    # Attendre la fin de la lecture du signal sur les haut-parleurs
    play_thread.join()