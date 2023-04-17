"""
COMNUMFIP2 - Module Python pour les communications numériques à Télécom Physique Strasbourg (spécifiquement pour la formation FIP EII).

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
    frequence = 1000  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    print(bits)
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
    return signal_ami

def NRZ_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 1000  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    print(bits)
    # Générer le signal NRZ
    temps_bit = np.linspace(0, duree_bit, int(duree_bit * framerate), endpoint=False)
    signal_nrz = np.zeros(0)
    for bit in bits:
        if bit == 0:
            signal_nrz = np.append(signal_nrz, -amplitude * np.ones_like(temps_bit))
        else:
            signal_nrz = np.append(signal_nrz, amplitude * np.ones_like(temps_bit))
    return signal_nrz

def manchester_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 1000  # Fréquence du signal en Hz
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
    return signal_manchester

def AMI_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 1000  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    print(bits)
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
    return signal_ami

def NRZ_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 1000  # Fréquence du signal en Hz
    duree_bit = 1/frequence  # Durée d'un bit en secondes
    # Convertir le message en une séquence de bits (0 et 1)
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    print(bits)
    # Générer le signal NRZ
    temps_bit = np.linspace(0, duree_bit, int(duree_bit * framerate), endpoint=False)
    signal_nrz = np.zeros(0)
    for bit in bits:
        if bit == 0:
            signal_nrz = np.append(signal_nrz, -amplitude * np.ones_like(temps_bit))
        else:
            signal_nrz = np.append(signal_nrz, amplitude * np.ones_like(temps_bit))
    return signal_nrz

def manchester_signal_generator(message):
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    frequence = 1000  # Fréquence du signal en Hz
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
    return signal_manchester

def bin2ascii(bits): # Séparer les bits en groupes de 8 
    bits_str =''.join(str(bit) for bit in bits)
    bytes_list = [bits_str[i:i+8] for i in range(0, len(bits_str), 8)] # Convertir chaque groupe de 8 bits en un caractère ASCII 
    ascii_str = ''.join([chr(int(byte, 2)) for byte in bytes_list]) 
    return ascii_str

"""
COMNUMFIP - Module Python pour les communications numériques à Télécom Physique Strasbourg (spécifiquement pour la formation FIP EII).

Ce programme est distribué sous licence CeCILL-B (www.cecill.info).
Copyright Université de Strasbourg 2013-2022 (2022-03-04)
Contributeur : vincent.mazet@unistra.fr
"""


import numpy as np
import numpy.random as rnd
import scipy.signal as signal
import matplotlib.pyplot as plt


def eyediag(t, x, T, alpha=.5, color="tab:blue"):
    
    """
    Diagramme de l'oeil.
    
    Entrées :
    t (array)      : temps
    x (array)      : signal
    T (scalar)     : durée d'un symbole
    alpha (scalar) : transparence (0,5 par défaut)
    
    Sortie :
    aucune
    """
    
    # % Détecte les instants auxquels le temps revient à -T/2
    t = t%T - T/2
    idx = np.flatnonzero(t[1:] < t[:-1])

    # Affichage des traces, séparément
    j = 0
    for i in idx:
        plt.plot(t[j:i+1], x[j:i+1], alpha=alpha, color=color)
        j = i+1
        

def rrc(t, V, a):
    
    """
    Impulsion en racine de cosinus surélevé (root raised cosine),
    pour une durée de symbole égale à 1.
    
    Entrées :
    t (array)  : temps
    V (scalar) : amplitude de l'impulsion
    a (scalar) : facteur de retombée (roll-off factor)
    
    Sortie :
    y (array) : impulsion en racine de cosinus surélevé
    """
    
    idx = (t==1/(4*a)) | (t==-1/(4*a)) | (t==0.)
    t = np.where(idx, t+1e-12, t)
    
    A = np.sin(np.pi*(1 - a)*t)
    B = 4*a*t * np.cos(np.pi*(1 + a)*t)
    C = np.pi*t*(1 - (4*a*t)**2)
    return V * ( A + B ) / C


def randmary(N,p):
    
    """
    Génération d'une séquence M-aire.
    
    Entrées :
    N (scalar) : taille de la séquence (nombre de symboles)
    P (array)  : probabilité des symboles (sa taille correspond à la taille de l'alphabet)
    
    Sortie :
    c (array) : séquence aléatoire M-aire où M = len(P).
    
    Exemples :
    
    # séquence binaire de taille 1000, symboles équiprobables :
    c1 = randmary(1000,[0.5, 0.5])
    
    # séquence binaire de taille 100, p("0") = 0.3, p("1") = 0.7 :
    c2 = randmary(100,[0.3, 0.7])
    
    # séquence 4-aire de taille 10, symboles équiprobables :
    c3 = randmary(10,np.ones(4)/4)
    """

    # Base
    M = len(p)
    
    # Normalisation des probabilités
    p = p / np.sum(p)
    
    # Fonction de répartition
    q = np.cumsum(p)
    
    # Vecteur aléatoire uniforme
    u = np.random.rand(N)
    
    # Matrice NxM des u et q
    U = np.tile(u,(M,1)).T
    Q = np.tile(q,(N,1))
    
    # Séquence de symboles
    c = np.sum(U>Q, axis=1)
    
    return c


def bin2mary(x,M):
    
    """
    Convertit une séquence binaire en séquence M-aire.
    Si la taille de x n'est pas multiple de log2(M), des "0" sont rajoutés à la fin de x.
    
    Entrées :
    x (array)  : séquence binaire
    M (scalar) : taille de l'alphabet de la séquence traduite (M est une puissance de 2)
    
    Sortie :
    y (array) : séquence M-aire
    """
    
    # Nombre de bits par symboles
    N = np.log2(M).astype("int")
    
    # Nombre de bits dans la séquence binaire x
    K = len(x)
    
    # Nombre de symboles dans la séquence M-aire y
    L = np.ceil(K/N).astype("int")
    
    # Rajoute des zéros en fin de z pour avoir un nombre de bits en puissance de N
    z = np.concatenate((x, np.zeros(N*L-K, dtype="int")))
    
    # Initialisation de la séquence de sortie
    y = np.array([], dtype="int")
    
    # Array des puissances
    powers = np.power(2,range(N))
    
    for i in range(0, K, N):
        m = z[i:i+N]
        c = np.sum(m*powers)
        y = np.append(y, c)
    
    return y


def mod_a(m, V, d):
    
    """
    Modulation mystère A.
    
    Entrées :
    m (array)    : séquence binaire
    V (scalaire) : amplitude de la forme d'onde
    d (scalaire) : durée de la forme d'onde
    
    Sorties :
    t (array) : vecteur temps
    x (array) : signal modulé
    """

    N = len(m)
    x = np.zeros(100*N)
    sgn = -1

    for n in range(N):
        i = 100*n + np.arange(100)
        if m[n] == 0:
            x[i] = 0
        elif m[n] == 1:
            sgn = -sgn
            x[i] = sgn*V * np.ones(100)

    t = np.arange(100*N)/100*d
    
    return t, x


def mod_b(m, V, d):
    
    """
    Modulation mystère B.
    
    Entrées :
    m (array)    : séquence binaire
    V (scalaire) : amplitude de la forme d'onde
    d (scalaire) : durée de la forme d'onde
    
    Sorties :
    t (array) : vecteur temps
    x (array) : signal modulé
    """

    N = len(m)
    x = np.zeros(100*N)
    t = np.arange(100)*d/100
    z = V * np.cos(2*np.pi*4/d*t)

    for n in range(N):
        i = 100*n + np.arange(100)
        if m[n] == 0:
            x[i] = -z
        elif m[n] == 1:
            x[i] = +z

    t = np.arange(100*N)/100*d
    
    return t, x


def mod_c(m, V, d):
    
    """
    Modulation mystère C.
    
    Entrées :
    m (array)    : séquence binaire
    V (scalaire) : amplitude de la forme d'onde
    d (scalaire) : durée de la forme d'onde
    
    Sorties :
    t (array) : vecteur temps
    x (array) : signal modulé
    """

    N = len(m)
    x = np.zeros(100*N)

    for n in range(N):
        i = 100*n + np.arange(100)
        if m[n] == 0:
            x[i] = V * np.concatenate((-np.ones(50), np.ones(50)))
        elif m[n] == 1:
            x[i] = V * np.concatenate((np.ones(50), -np.ones(50)))

    t = np.arange(100*N)/100*d
    
    return t, x


def mod_d(m, V, d):
    
    """
    Modulation mystère D.
    
    Entrées :
    m (array)    : séquence binaire
    V (scalaire) : amplitude de la forme d'onde
    d (scalaire) : durée de la forme d'onde
    
    Sorties :
    t (array) : vecteur temps
    x (array) : signal modulé
    """

    N = len(m)
    x = np.zeros(100*N)

    for n in range(N):
        i = 100*n + np.arange(100)
        if m[n] == 0:
            x[i] = -V * np.ones(100)
        elif m[n] == 1:
            x[i] = V * np.ones(100)

    t = np.arange(100*N)/100*d
    
    return t, x


def mod_e(m, V, d):
    
    """
    Modulation mystère E.
    
    Entrées :
    m (array)    : séquence binaire
    V (scalaire) : amplitude de la forme d'onde
    d (scalaire) : durée de la forme d'onde
    
    Sorties :
    t (array) : vecteur temps
    x (array) : signal modulé
    """
    
    f = 4/d
    N = len(m)
    x = np.zeros(100*N)
    t = np.arange(100)*d/100
    
    constellation = [
        {'a': -3, 'b': +3},
        {'a': -1, 'b': +3},
        {'a': -3, 'b': +1},
        {'a': -1, 'b': +1},
        {'a': +3, 'b': +3},
        {'a': +1, 'b': +3},
        {'a': +3, 'b': +1},
        {'a': +1, 'b': +1},
        {'a': -3, 'b': -3},
        {'a': -1, 'b': -3},
        {'a': -3, 'b': -1},
        {'a': -1, 'b': -1},
        {'a': +3, 'b': -3},
        {'a': +1, 'b': -3},
        {'a': +3, 'b': -1},
        {'a': +1, 'b': -1}
    ]
    
    for n in range(N):
        i = 100*n + np.arange(100)
        x[i] = constellation[m[n]]['a']*V*np.cos(2*np.pi*f*t) + constellation[m[n]]['b']*V*np.sin(2*np.pi*f*t)
    
    t = np.arange(100*N)/100*d
    
    return t, x


def mod_rrc(m, V, T, a):
    
    """
    Modulation NRZ en racine de cosinus surélevé
    
    Entrées :
    m (array)  : séquence binaire
    V (scalar) : amplitude de la forme d'onde
    T (scalar) : durée de la forme d'onde
    a (scalar) : coefficient de retombée (roll-off factor)
    
    Sorties :
    t (array) : vecteur temps
    x (array) : signal modulé
    """
    
    N = len(m)
    L = 100
    m = 2*np.array(m) - 1
    t = np.arange(L*N)/L*T
    x = np.zeros(L*N)
    for n in range(N):
        x += m[n] * rrc((t/T-n)-.5, V, a)
        
    return t, x


def channel(x,fc,s,T):
    
    """
    Simule un canal de transmission en renvoyant le signal y = x*g + b en sortie du canal,
    où g est le réponse impulsionnelle d'un filtre passe-bas, b un bruit blanc gaussien et * représente la convolution.
    
    Entrées :
    x (array)   : signal émis
    fc (scalar) : fréquence de coupure du filtre g
    s (scalar)  : écart-type du bruit b
    T (scalar)  : durée d'un bit
    
    Sortie :
    y (array) : signal transmis via le canal
    """
    
    fe = 100/T
    
    # Filtre passe-bas (seulement si canal non idéal)
    if fc<fe/2:
        num, den = signal.ellip(8, 1, 80, fc*2/fe)
        x = signal.lfilter(num, den, x)
    
    # Bruit
    b = rnd.normal(0, s, x.shape)
    
    return x + b


def rleenc(msg):

    """
    Compression RLE (run length encoding).
    
    Entrée :
    msg (array) : séquence de symboles à compresser
    
    Sortie :
    code (array) : séquence compressée en RLE
    
    Exemple :
    
    from skimage.io import imread
    mg = imread("image.png")         # Charge l'image image.png
    code = rleenc(img.ravel())       # .ravel() permet de vectoriser l'image
                                     # pour en faire un tableau à une seule dimension
    """
    
    # Initialisation avec le premier élément
    code = []
    nb = 1
    prev_m = msg[0]
    
    # Boucle sur les éléments suivants
    for m in msg[1:]:
        
        if (m != prev_m) or (nb == 255):
            code.append(prev_m)
            code.append(nb)
            nb = 1
            prev_m = m
        
        else:
            nb += 1
            
    # Ajout des derniers éléments
    code.append(prev_m)
    code.append(nb)
            
    return code


def rledec(code):

    """
    Décompression RLE (run length encoding).
    
    Entrée :
    code (array) : séquence compressée en RLE
    
    Sortie :
    msg (array) : séquence de symboles décompressée
    
    Exemple :
    
    from numpy import reshape
    msg = rledec(code)               # Effectue la décompression RLE
    img = reshape(msg, (M,N))        # Si c'est une image qui est attendue,
                                     # transforme la séquence msg en image de taille M×N    """
    
    N = len(code)
    msg = np.array([])
    
    # Boucle sur les éléments du code
    for i in range(0,N,2):
        
        val = code[i]
        num = code[i+1]
        
        msg = np.append(msg, [val]*num)
        
    return msg


def sample_and_threshold(x, S):
    
    """
    Échantillonne à la période T et compare au seuil S le signal x,
    pour retourner une séquence binaire
    
    Entrées :
    x (array)  : signal
    T (scalar) : période d'échantillonnage (= durée d'un bit)
    S (scalar) : seuil à appliquer
    
    Sortie :
    y (array) : séquence binaire
    """
    
    L = 4
    idx = range(int(L/2), len(x), L)
    y = np.where(x[idx]>S, 1, 0)
    
    return y