from BME280 import *
from DRV8833 import *
from VL6180X import *
from micropython import const
from machine import Pin
from machine import Timer
from math import cos
from math import sin
from math import fmod
import math
from machine import SD
import os

#----------------- INITIALISATION ----------------------------------------------

#On crée et lancle le chrono
chrono = Timer.Chrono()
chrono.start()

#Carte SD
try:
    sd = SD()
except OSError:
    print('----------------------------------')
    print('Veuillez insérer une carte SD !')
    print('----------------------------------')

os.mount(sd, '/sd')
time.sleep(0.02)
f = open('/sd/raw_data.csv', 'w')
time.sleep(0.02)
print('Nouveau Fichier créé dans la carte SD')

#On écris la première ligne du fichier
f.write('pid ; x ; y ; t ; d ; a ; c ; l ; p ; h\n')
f.close()

#Capteur distance/luminosité
VL6180X_CE_Pin = 'P6'
VL6180X_I2C_adr_defaut = const(0x29)
VL6180X_GPIO_CE_Pin = Pin(VL6180X_CE_Pin, mode=Pin.OUT)
VL6180X_GPIO_CE_Pin.value(1)
i2c = I2C(0, I2C.MASTER, baudrate=400000)

capteur_d_l_VL6180X = VL6180X(VL6180X_I2C_adr_defaut, i2c)

#Capteur de température
bus_i2c = I2C(0)

Id_BME280 = bus_i2c.readfrom_mem(BME280_I2C_ADR, BME280_CHIP_ID_ADDR, 1)

capteur_BME280 = BME280(BME280_OVERSAMPLING_16X,
BME280_OVERSAMPLING_16X,
BME280_OVERSAMPLING_16X,
BME280_FILTER_COEFF_2,
BME280_STANDBY_TIME_125_MS,
BME280_NORMAL_MODE,
BME280_I2C_ADR,
bus_i2c)

capteur_BME280.Calibration_Param_Load()

temperature = capteur_BME280.read_temp()
humidite = capteur_BME280.read_humidity()
pression = capteur_BME280.read_pression()

#Moteurs
DRV8833_Sleep_pin = 'P5'
DRV8833_AIN1 = 'P11'
DRV8833_AIN2 = 'P12'
DRV8833_BIN1 = 'P21'
DRV8833_BIN2 = 'P22'

Moteur_Gauche = DRV8833(DRV8833_AIN1, DRV8833_AIN2, DRV8833_Sleep_pin, 1, 500, 0, 1)
Moteur_Droit = DRV8833(DRV8833_BIN1, DRV8833_BIN2, DRV8833_Sleep_pin, 1, 500, 2, 3)

def reculer(vitesse=1):
    Moteur_Gauche.Cmde_moteur(SENS_HORAIRE, vitesse)
    Moteur_Droit.Cmde_moteur(SENS_ANTI_HORAIRE, vitesse)

def avancer(vitesse=1):
    #Correction du au carrossage de la roue droite
    Moteur_Gauche.Cmde_moteur(SENS_ANTI_HORAIRE, vitesse - vitesse*0.05)
    Moteur_Droit.Cmde_moteur(SENS_HORAIRE, vitesse)

def tourner_gauche(vitesse=1):
    Moteur_Gauche.Cmde_moteur(SENS_HORAIRE, vitesse)
    Moteur_Droit.Cmde_moteur(SENS_HORAIRE, vitesse)

def tourner_droit(vitesse=1):
    Moteur_Gauche.Cmde_moteur(SENS_ANTI_HORAIRE, vitesse)
    Moteur_Droit.Cmde_moteur(SENS_ANTI_HORAIRE, vitesse)

RESOLUTION_CODEUR_ROUE = const(1400)

ticks_Md_EncA = 0

ticks_Md_EncB = 0
ticks_Mg_EncA = 0

ticks_Mg_EncB = 0
ticks_Md_EncA_old = 0
ticks_Md_EncB_old = 0
ticks_Mg_EncA_old = 0
ticks_Mg_EncB_old = 0

#Le rayon de la roue en cm
RAYON_ROUE = 5.3

#Entraxe des roues en cm
L = 8.3

x_pos = 0.0
y_pos = 0.0
theta = 0.0

def IT_Moteur_droit_EncodeurA(arg):
    global ticks_Md_EncA

def IT_Moteur_droit_EncodeurB(arg):
    global ticks_Md_EncB
    ticks_Md_EncB += 1

def IT_Moteur_gauche_EncodeurA(arg):
    global ticks_Mg_EncA
    ticks_Mg_EncA += 1

def IT_Moteur_gauche_EncodeurB(arg):
    global ticks_Mg_EncB
    ticks_Mg_EncB += 1

def IT_Alarme_Odometrie(arg):
    global ticks_Md_EncA
    global ticks_Md_EncB
    global ticks_Mg_EncA
    global ticks_Mg_EncB
    global ticks_Md_EncA_old
    global ticks_Md_EncB_old
    global ticks_Mg_EncA_old
    global ticks_Mg_EncB_old
    global Odometrie_Flag
    global x_pos
    global y_pos
    global theta
    dif_ticks_Md_EncA = ticks_Md_EncA - ticks_Md_EncA_old
    dif_ticks_Md_EncB = ticks_Md_EncB - ticks_Md_EncB_old
    dif_ticks_Mg_EncA = ticks_Mg_EncA - ticks_Mg_EncA_old
    dif_ticks_Mg_EncB = ticks_Mg_EncB - ticks_Mg_EncB_old

    delta_l_roue_gauche = 2 * math.pi * RAYON_ROUE * (dif_ticks_Mg_EncA + dif_ticks_Mg_EncB) * 0.5 / RESOLUTION_CODEUR_ROUE

    delta_l_roue_droite = 2 * math.pi * RAYON_ROUE * (dif_ticks_Md_EncA + dif_ticks_Md_EncB) * 0.5 / RESOLUTION_CODEUR_ROUE

    delta_l_moyen = 0.5 * (delta_l_roue_gauche + delta_l_roue_droite)
    delta_x_pos = delta_l_moyen * cos(theta)
    delta_y_pos = delta_l_moyen * sin(theta)
    delta_theta = (delta_l_roue_droite -
    delta_l_roue_gauche) / L # En radian
    x_pos += delta_x_pos
    y_pos += delta_y_pos
    theta += delta_theta
    theta = fmod(theta, math.pi)
    ticks_Md_EncA_old = ticks_Md_EncA
    ticks_Md_EncB_old = ticks_Md_EncB
    ticks_Mg_EncA_old = ticks_Mg_EncA
    ticks_Mg_EncB_old = ticks_Mg_EncB
    Odometrie_Flag = True

Mot_Droit_EncodeurA = Pin('P13', mode = Pin.IN, pull=Pin.PULL_UP)
Mot_Droit_EncodeurB = Pin('P15', mode = Pin.IN, pull=Pin.PULL_UP)
Mot_Gauche_EncodeurA = Pin('P17', mode = Pin.IN, pull=Pin.PULL_UP)
Mot_Gauche_EncodeurB = Pin('P18', mode = Pin.IN, pull=Pin.PULL_UP)

Mot_Droit_EncodeurA.callback(Pin.IRQ_RISING | Pin.IRQ_FALLING, IT_Moteur_droit_EncodeurA)
Mot_Droit_EncodeurB.callback(Pin.IRQ_RISING | Pin.IRQ_FALLING, IT_Moteur_droit_EncodeurB)
Mot_Gauche_EncodeurA.callback(Pin.IRQ_RISING | Pin.IRQ_FALLING, IT_Moteur_gauche_EncodeurA)
Mot_Gauche_EncodeurB.callback(Pin.IRQ_RISING | Pin.IRQ_FALLING, IT_Moteur_gauche_EncodeurB)

Timer.Alarm(IT_Alarme_Odometrie, ms=10, periodic=True)

print('-------------------------')
print('FIN D\'INITIALISATION')
print('-------------------------')

#----------------- INSTRUCTIONS ------------------------------------------------

k = 0
pid = 1
a = -5

while True:
    k += 1
    f = open('/sd/raw_data.csv', 'a')

    angle = (theta*180.0 / math.pi)
    distance = capteur_d_l_VL6180X.range_mesure()
    luminosite = capteur_d_l_VL6180X.ambiant_light_mesure()
    temperature = capteur_BME280.read_temp()
    pression = capteur_BME280.read_pression()
    humidite = capteur_BME280.read_humidity()
    temps = chrono.read()

    #La distance n'est pas fiable car max = 255
    if distance < 255:
        info = str(pid) + ';' + str(x_pos) + ';' + str(y_pos) + ';' + str(temps) + ';' + str(distance) + ';' + str(angle)+ ';' + str(temperature) + ';' + str(luminosite) + ';' + str(pression) + ';' + str(humidite) + '\n'
    else:
        info = str(pid) + ';' + str(x_pos) + ';' + str(y_pos) + ';' + str(temps) + ';' + '<null>' + ';' + str(angle)+ ';' + str(temperature) + ';' + str(luminosite) + ';' + str(pression) + ';' + str(humidite) + '\n'

    #Si on est à plus de 20cm
    if distance > 200:
        avancer(.4)

        #Si on est 4s plus tard que le dernier enregistrement sur la SD
        if (int(temps)%4 == 0) and (k-a > 7):
            a = k
            f.write(info)
            f.close()
            pid += 1

    else:
        #Si le numéro de boucle est pair
        if k%2 == 0:
            reculer(.5)
            time.sleep(1)
            tourner_droit(.5)

            #Si on est 4s plus tard que le dernier enregistrement sur la SD
            if (int(temps)%4 == 0) and (k-a > 7):
                a = k
                f.write(info)
                f.close()
                pid += 1
        #Si le numéro de boucle est impair
        else:
            reculer(.5)
            time.sleep(1)
            tourner_gauche(.5)

            #Si on est 4s plus tard
            if (int(temps)%4 == 0) and (k-a > 7):
                a = k
                f.write(info)
                f.close()
                pid += 1
