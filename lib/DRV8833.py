# Gestion des moteurs CC à l'aide du double pont en H DRV8833
# Validé le 12.09.2018
from micropython import const
from machine import I2C
from machine import Pin
from machine import PWM
import time

# Définition sens de rotation des moteurs
SENS_HORAIRE = const(1)
SENS_ANTI_HORAIRE = const(2)

class DRV8833 :
    def __init__  (self, In1_pin, In2_pin, sleep_pin, timer_number, freq, num_channel_pwm_In1, num_channel_pwm_In2, **kwargs) :
        # IN1_pin : entrée PWM 1 DRV8833
        # IN2_pin : entrée PWM 2 DRV8833
        # sleep_pin : SLP pin pour désactiver les ponts en H du DRV8833
        # timer_number : dans [0,1,2,3]. Choix du timer utilisé pour générer le signal pwm
        # freq : fréquence du signal pwm
        # num_channel_pwm_In1 : numéro de l'Id du canal PWM associé à la broche In1_pin
        # num_channel_pwm_In2 : numéro de l'Id du canal PWM associé à la broche In2_pin

        self.DRV883_Sleep_Pin = Pin(sleep_pin, mode = Pin.OUT) # Initialiser la broche sleep_pin pour gérer le DRV8833
        self.DRV883_Sleep_Pin.value(0) # Désactive le driver DRV8833
        if timer_number not in [0,1,2,3] :
            raise ValueError(
                'Unexpected timer_number value {0}.'.format(timer_number))
        self.pwm = PWM(timer_number, frequency = freq) # Utiliser le timer n° timer_number en PWM avec une fréquence de base de freq Hz
        self.DRV8833_Pwm_In1 = self.pwm.channel(num_channel_pwm_In1, pin = In1_pin, duty_cycle = 0.0)
        self.DRV8833_Pwm_In2 = self.pwm.channel(num_channel_pwm_In2, pin = In2_pin, duty_cycle = 0.0)
        time.sleep(0.05)
#---------------------------------------------------------------------------
# Commande d'un moteur : paramètres : sens de rotation et vitesse dans [0.0;1.0]
    def Cmde_moteur (self, sens, vitesse) :
        self.DRV883_Sleep_Pin.value(0) # Désactive le driver DRV8833
        if vitesse < 0.0 :
            vitesse = 0.0
        elif vitesse > 1.0 :
            vitesse = 1.0
        elif vitesse <= 0.3 :
            v_init = 0.45
            Delta_v = (v_init - vitesse) / 10
            self.DRV883_Sleep_Pin.value(1) # Activer le driver DRV8833
            for i in range(10) :
                if sens == SENS_HORAIRE : # forward
                    self.DRV8833_Pwm_In1.duty_cycle(v_init - (i+1) * Delta_v) # Rapport cyclique à vitesse % sur IN1
                    self.DRV8833_Pwm_In2.duty_cycle(0.0) # Rapport cyclique à 0.0 sur IN2 (soit 0%)
                    time.sleep (0.005)
                elif sens == SENS_ANTI_HORAIRE : # reverse
                    self.DRV8833_Pwm_In1.duty_cycle(0.0) # Rapport cyclique à 0.0 sur IN1
                    self.DRV8833_Pwm_In2.duty_cycle(v_init - (i+1) * Delta_v) # Rapport cyclique à vitesse % sur IN2
                    time.sleep (0.005)
        else :
            self.DRV883_Sleep_Pin.value(1) # Activer le driver DRV8833.value(1) # Activer le driver DRV8833
            if sens == SENS_HORAIRE : # forward
                self.DRV8833_Pwm_In1.duty_cycle(vitesse) # Rapport cyclique à vitesse % sur IN1
                self.DRV8833_Pwm_In2.duty_cycle(0.0) # Rapport cyclique à 0.0 sur IN2 (soit 0%)
            elif sens == SENS_ANTI_HORAIRE : # reverse
                self.DRV8833_Pwm_In1.duty_cycle(0.0) # Rapport cyclique à 0.0 sur IN1
                self.DRV8833_Pwm_In2.duty_cycle(vitesse) # Rapport cyclique à vitesse % sur IN2
                time.sleep (0.005)
#---------------------------------------------------------------------------
# Définitions des mouvements de base de la plateforme robotique
    def Arret_moteur (self) :
        self.DRV883_Sleep_Pin.value(1) # Activer le driver DRV8833
        self.DRV8833_Pwm_In1.duty_cycle(0.0) # Rapport cyclique à 0.0 sur IN1
        self.DRV8833_Pwm_In1.duty_cycle(0.0) # Rapport cyclique à 0.0 sur IN1
        self.DRV883_Sleep_Pin.value(0) # Désactive le driver DRV8833
