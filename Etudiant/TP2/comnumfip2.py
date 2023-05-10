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
import subprocess
import sounddevicecustom as sd_cust
from scipy.signal import find_peaks




def send_signal_to_canal(signal):
    
    """
    Envoie un signal audio au canal de sortie.
    
    Entrées :
    - signal : numpy.ndarray contenant le signal audio à envoyer.
    Sorties : Aucune
    """
    
    sd.play(signal, 44100)
    sd.wait()
    
def add_start_and_stop_bit(bits):
    
    """
    Ajoute des bits de synchronisation, de début et de fin à une trame de bits.
    
    Entrée :
    - bits : numpy.ndarray contenant la trame de bits à laquelle ajouter les bits de synchronisation, de début et de fin.
    Sortie :
    - numpy.ndarray contenant la trame de bits avec les bits de synchronisation, de début et de fin ajoutés.    
    """
    
    #bit de synchronisation pour que le signal soit détecté
    bit_sync         =  np.zeros(441) #441 bits pour faire 100 ms
    bit_sync[1:]     =  2
    bit_sync[:10]    =  3
    #bit de start
    bit_start        =  np.zeros(441) #441 bits pour faire 100 ms
    bit_start[1:]    =  2
    bit_start[0]     =  3
    #bit de stop 
    bit_stop         =  np.zeros(441) #441 bits pour faire 100 ms
    bit_stop[0:441]  =  2
    bit_stop[-1]     =  3
    #ajout des bits sur la trame à envoyer
    bits             = np.insert(bits,0,bit_start) # first insert start bit then stop
    bits             = np.insert(bits,0,bit_sync)
    bits             = np.append(bits, bit_stop)
    return bits

def string_to_ascii(message):
    
    """
    Convertit une chaîne de caractères en une séquence de bits en ASCII.
    
    Entrée :
    - message : chaîne de caractères à convertir en bits.

    Sortie :
    - numpy.ndarray contenant la séquence de bits en ASCII.    
    """
    
    bits = np.unpackbits(np.array([ord(c) for c in message], dtype=np.uint8))
    return bits
    

def ami_signal_generator(bits):
    
    """
    Génère un signal AMI à partir d'une séquence de bits donnée.

    Entrée :
    - bits : séquence de bits à moduler.

    Sortie :
    - numpy.ndarray contenant le signal AMI modulé correspondant à la séquence de bits.   
    """
    
    # Paramètres du signal AMI
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    duree_motif = 1/4410  # Durée d'un bit en secondes
    # Générer le signal AMI
    temps_bit = np.linspace(0, duree_motif, int(duree_motif * framerate), endpoint=False)
    signal_ami = np.zeros(0)
    #ajout des bits de sync start et stop
    bits = add_start_and_stop_bit(bits)
    #génération du message à envoyer 
    for bit in bits:
        if bit == 0:
            signal_ami = np.append(signal_ami, 0 * np.ones_like(temps_bit))
        elif bit ==1:
            signal_ami = np.append(signal_ami, amplitude * np.ones_like(temps_bit))
            amplitude = -amplitude
        elif bit == 2:
            signal_ami = np.append(signal_ami,  0* np.ones_like(temps_bit))
        elif bit == 3:
            signal_ami = np.append(signal_ami,  np.abs(amplitude)* np.ones_like(temps_bit))
             # Ajouter des zéros au début et à la fin du signal
    #Ajout de zéros au début et à la fin du signal pour éviter les erreurs
    signal_ami = np.pad(signal_ami, (int(framerate/2), int(framerate/2)), 'constant')
    return signal_ami


def nrz_signal_generator(bits):
    
    """
    Génère un signal NRZ à partir d'une séquence de bits donnée.

    Entrée :
    - bits : séquence de bits à moduler.

    Sortie :
    - numpy.ndarray contenant le signal NRZ modulé correspondant à la séquence de bits.   
    """
    
    # Paramètres du signal NRZ
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5 # Amplitude du signal
    duree_motif = 1/4410  # Durée d'un bit en secondes
    # Générer le signal NRZ
    temps_bit = np.linspace(0, duree_motif, int(duree_motif * framerate), endpoint=False)
    signal_nrz = np.zeros(0)
    #ajout des bits de sync start et stop
    bits = add_start_and_stop_bit(bits)
    #génération du message à envoyer 
    for bit in bits:
        if bit == 0:
            signal_nrz = np.append(signal_nrz, -amplitude * np.ones_like(temps_bit))
        elif bit == 1:
            signal_nrz = np.append(signal_nrz,  amplitude * np.ones_like(temps_bit))
        elif bit == 2:
            signal_nrz = np.append(signal_nrz,  0* np.ones_like(temps_bit))
        elif bit == 3:
            signal_nrz = np.append(signal_nrz,  amplitude* np.ones_like(temps_bit))
    #Ajout de zéros au début et à la fin du signal pour éviter les erreurs
    signal_nrz = np.pad(signal_nrz, (int(framerate/2), int(framerate/2)), 'constant')
    return signal_nrz

def manchester_signal_generator(bits):
    
    """
    Génère un signal Manchester à partir d'une séquence de bits donnée.

    Entrée :
    - bits : séquence de bits à moduler.

    Sortie :
    - numpy.ndarray contenant le signal Manchester modulé correspondant à la séquence de bits.   
    """
    
    # Paramètres du signal Manchester
    framerate = 44100 # Fréquence d'échantillonnage en Hz
    amplitude = 0.5   # Amplitude du signal
    duree_motif = 1/4410  # Durée d'un bit en secondes
    duree_motif = duree_motif/2
    # Générer le signal Manchester
    temps_bit = np.linspace(0, duree_motif, int((duree_motif) * framerate), endpoint=False)
    signal_manchester = np.zeros(0)
    #ajout des bits de sync start et stop
    bits = add_start_and_stop_bit(bits)
    #génération du message à envoyer 
    for bit in bits:
        if bit == 0:
            signal_manchester = np.append(signal_manchester, amplitude * np.ones_like(temps_bit))
            signal_manchester = np.append(signal_manchester, -amplitude * np.ones_like(temps_bit))
        elif bit == 1:
            signal_manchester = np.append(signal_manchester, -amplitude * np.ones_like(temps_bit))
            signal_manchester = np.append(signal_manchester, amplitude * np.ones_like(temps_bit))
        elif bit == 2:
            signal_manchester = np.append(signal_manchester,  0* np.ones_like(temps_bit))
            signal_manchester = np.append(signal_manchester,  0* np.ones_like(temps_bit))
        elif bit == 3:
            signal_manchester = np.append(signal_manchester,  amplitude* np.ones_like(temps_bit))
            signal_manchester = np.append(signal_manchester,  amplitude* np.ones_like(temps_bit))
    #Ajout de zéros au début et à la fin du signal pour éviter les erreurs
    signal_manchester = np.pad(signal_manchester, (int(framerate/2), int(framerate/2)), 'constant')
    return signal_manchester
    
    


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
 
    
def bin2ascii(bits): 

    """
    Sépare le tableau de bit en groupe de 8 puis converti chaque octet en un caractère ascii.
    
    Entrées :
    bits (array)  : tableau de bits
    
    Sortie :
    ascii_str (string) : chaîne de caractère ascii
    """
    
    bits = bits.flatten()
    bits_str =''.join(str(bit) for bit in bits)
    bytes_list = [bits_str[i:i+8] for i in range(0, len(bits_str), 8)] # Convertir chaque groupe de 8 bits en un caractère ASCII 
    ascii_str = ''.join([chr(int(byte, 2)) for byte in bytes_list]) 
    return ascii_str

def sample_and_threshold_Manchester(x, S,sample_time_shift):
    
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
    
    shifted_index = int((sample_time_shift)*10/(1/4410))
    L = 5
    idx = range(int(L/2)-(np.abs(shifted_index)), len(x), L)
    idx = idx[::2]
    y = np.where(x[idx]>S, 0, 1) 
    #plt.figure()
    #plt.title("Manchester")
    #plt.plot(x)
    #plt.plot(idx,x[idx],'.')
    #plt.xlim(0,100)
    #plt.show()
    return y 

def sample_and_threshold_NRZ(x, S,sample_time_shift):
    
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
    
    shifted_index = int((sample_time_shift)*10/(1/4410))
    L = 10 
    idx = range(int(L/2)-(np.abs(shifted_index)), len(x), L)
    y = np.where(x[idx]>S, 1, 0) 
    #plt.figure()
    #plt.title("NRZ")
    #plt.plot(x)
    #plt.plot(idx,x[idx],'.')
    #plt.xlim(0,100)
    #plt.show()
    return y


def sample_and_threshold_AMI(x, S,sample_time_shift):
    
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
    
    shifted_index = int((sample_time_shift)*10/(1/4410))
    L = 10
    idx = range(int(L/2)-(np.abs(shifted_index)), len(x), L)
    y = np.where((x[idx]>S) | (x[idx]<-S), 1, 0) 
    #plt.figure()
    #plt.title("AMI")
    #plt.plot(x)
    #plt.plot(idx,x[idx],'.')
    #plt.xlim(0,100)
    #plt.show()
    return y

def init_volume():
    
    """
    Permet de régler le volume de l'ordinateur pour assurer une bonne acquisition.
    
    Entrées :
    aucune
    
    Sortie :
    aucune
    """
    
    val = 100
    val = float(int(val))
    proc = subprocess.Popen('/usr/bin/amixer sset Master ' + str(val) + '%', shell=True, stdout=subprocess.PIPE)
    proc.wait()
    val = 50
    val = float(int(val))
    proc = subprocess.Popen('/usr/bin/amixer sset Capture ' + str(val) + '%', shell=True, stdout=subprocess.PIPE)
    proc.wait()

def check_config_transmitter(): 
    
    """
    Envoi un signal connu au récepteur pour vérifier si la configuration est correcte
    
    Entrées :
    aucune
    
    Sortie :
    aucune
    """
    
    message = "test config ok"
    signal  = nrz_signal_generator(message)   
    send_signal_to_canal(signal)


def check_config_receiver(): 
    
    """
    Entrées :
    aucune
    
    Sortie :
    aucune
    """
    
    #acquisition du signal 
    signal = acquire_signal(44100,3)
    n = np.arange(0,len(signal))/44100
    
    #affichage pour vérification visuelle
    plt.figure(figsize=(20,5))
    plt.title("Signal acquis pour vérification")
    plt.xlabel("temps (s)")
    plt.ylabel("Amplitude du signal (V)")
    plt.plot(n,signal)
    plt.show()
    
    #vérification du contenu du message
    start_sample = _find_start_bit(signal)
    stop_sample  = _find_stop_bit(signal)
    
    #supprime toutes les valeurs 100ms après et avant les bits de start et stop pour ne garder que le message
    signal = signal[:stop_sample-4400]
    signal = signal[start_sample+4400::]
    
    message_binaire=sample_and_threshold_NRZ(signal,0.5)
    message_str = bin2ascii(message_binaire)
    
    if message_str == "test config ok":
        print("Le test semble correct. Veuillez vérifier l'allure du signal.")
    else :
        print("Erreur ajuster les paramètres manuellement")
    
  

def _find_start_bit(signal,threshold):

    """
    Permet de trouver l'échantillon exact où le bit de start d'un signal de communication numérique se termine
    Entrée :
    Signal de communication numérique avec un bit de start

    Sortie :
    Valeur de l'échantillon à laquelle le bit de start s'est terminé.
    """

    #recherche tout les piques positif est négatif du signal au dessus du seuil de 0.5
    if signal.ndim == 2:
        positive_peaks,_ = find_peaks( signal[:,0], height=threshold) 
        negative_peaks,_ = find_peaks(-signal[:,0], height=threshold) 
    else:
        positive_peaks,_ = find_peaks( signal, height=threshold) 
        negative_peaks,_ = find_peaks(-signal, height=threshold) 


    #cherche l'échantillon exact ou le bit de start c'est terminé
    n = np.arange(0,len(signal))
    if positive_peaks[0]>negative_peaks[0]:
        for i in range(negative_peaks[0],negative_peaks[0]+len(negative_peaks),1):
            if signal[i] > -threshold :
                return n[i-1]
    else:
        for i in range(positive_peaks[0],positive_peaks[0]+len(negative_peaks),1):
            if signal[i] < threshold :
                return n[i-1]

                
def _find_stop_bit(signal,threshold):
    
    """
    Permet de trouver l'échantillon exact où le bit de stop d'un signal de communication numérique commence
    Entrée :
    Signal de communication numérique avec un bit de stop.

    Sortie :
    Valeur de l'échantillon à laquelle le bit de stop a commencé
    """
    
    #recherche tout les piques positif est négatif du signal au dessus du seuil de 0.5
    if signal.ndim == 2:
        positive_peaks,_ = find_peaks( signal[:,0], height=threshold) 
        negative_peaks,_ = find_peaks(-signal[:,0], height=threshold) 
    else:
        positive_peaks,_ = find_peaks( signal, height=threshold) 
        negative_peaks,_ = find_peaks(-signal, height=threshold) 

    #cherche l'échantillon exact ou le bit de stop a commencé
    n = np.arange(0,len(signal))
    last_negative_peak = len(negative_peaks)
    last_positive_peak = len(positive_peaks)
    if positive_peaks[last_positive_peak-1]>negative_peaks[last_negative_peak-1]:
        for i in range(positive_peaks[last_positive_peak-1]-10,positive_peaks[last_positive_peak-1],1):
            if signal[i] > threshold :
                return n[i]
    else:
        for i in range(negative_peaks[last_negative_peak-1]-10,negative_peaks[last_negative_peak-1],1):
            if signal[i] < -threshold :
                return n[i]

def acquire_signal(fe: int, duration: float):
    
    """
    Acquisition du signal en communication numérique

    Entrées :
    - fe (int) : Taux d'échantillonnage de l'acquisition.
    - duration (float) : Durée de l'acquisition en secondes.

    Sortie :
    - numpy.ndarray : Signal acquis.
    """
    
    # Acquisition du signal.
    signal = sd_cust.rec(int(duration*fe), samplerate=fe, channels=1)
    sd_cust.wait()

    # Suppression des premières valeurs si le bit de synchronisation est encore présent.
    signal = signal[2200::]

    t = np.arange(0,len(signal))/44100

    return t,signal


def get_message_from_signal(signal,threshold):
    
    """
    Récupère le message à partir du signal
    Entrées :
    - signal : Signal dans lequel le message est recherché.
    - threshold : Seuil de détection des bits de start et de stop.

    Sortie :
    - numpy.ndarray : Signal contenant uniquement le message.
    """
    
    start_sample = _find_start_bit(signal,threshold)
    stop_sample  = _find_stop_bit(signal,threshold)


    #affichage du bit de start
    #n = np.arange(0,len(signal))/44100
    #plt.figure(figsize=(10,5))
    #plt.plot(n,signal)
    #plt.plot(n[start_sample], signal[start_sample], "o") 
    #plt.xlim(0.030,0.06)
    #plt.show()

    #supprime toutes les valeurs 100ms après et avant les bits de start et stop pour ne garder que le message
    signal = signal[:stop_sample-4400]
    signal = signal[start_sample+4400::]

    t = np.arange(0,len(signal))/44100
    return t,signal