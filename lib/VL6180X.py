# Gestion des capteurs de distance et luminosité VL6180X
# Mesure de distance sur la plage [1.0; 180] mm
# Mesure de luminosité en Lux

# Ressources carte Wipy 3.0
    # Bus I2C
        # P9 : SDA
        # P10 : SCL

from micropython import const
from machine import I2C
from machine import Pin
import time

# Constantes symboliques pour VL6180X
VL6180X_ID_MODEL = const(0xB4)          # ID VL6180X quelque soit l'@I2C

# Rq : par défaut, la broche GPIO0/CE du capteur VL6180X est à 0 => capteur inhibé
# Adresse des registres (cf. p 49-50 doc. ST)
VL6180X_REG_IDENTIFICATION_MODEL_ID       = const(0x000)
VL6180X_REG_SYSTEM_INTERRUPT_CONFIG       = const(0x014)
VL6180X_REG_SYSTEM_INTERRUPT_CLEAR        = const(0x015)
VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET     = const(0x016)
VL6180X_REG_SYSRANGE_START                = const(0x018)
VL6180X_REG_SYSALS_START                  = const(0x038)
VL6180X_REG_SYSALS_ANALOGUE_GAIN          = const(0x03F)
VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI  = const(0x040) # 2 octets
VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO  = const(0x041)
VL6180X_REG_RESULT_ALS_VAL                = const(0x050) # 2 octets
VL6180X_REG_RESULT_RANGE_VAL              = const(0x062)
VL6180X_REG_RESULT_RANGE_STATUS           = const(0x04d)
VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO  = const(0x04f)
VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS      = const(0x212)

# Facteur de gain pour mesure ALS - Ambient Light Sensing (cf. p 39 et 67 doc. ST)
ALS_GAIN_1         = const(0x46)
ALS_GAIN_1_25      = const(0x45)
ALS_GAIN_1_67      = const(0x44)
ALS_GAIN_2_5       = const(0x43)
ALS_GAIN_5         = const(0x42)
ALS_GAIN_10        = const(0x41)
ALS_GAIN_20        = const(0x40)
ALS_GAIN_40        = const(0x47)

# Code d'erreur (cf. p27 doc. ST)
ERROR_NONE         = const(0)
ERROR_SYSERR_1     = const(1)
ERROR_SYSERR_5     = const(5)
ERROR_ECEFAIL      = const(6)
ERROR_NOCONVERGE   = const(7)
ERROR_RANGEIGNORE  = const(8)
ERROR_SNR          = const(11)
ERROR_RAWUFLOW     = const(12) # Range value underflow < 0
ERROR_RAWOFLOW     = const(13)
ERROR_RANGEUFLOW   = const(14) # Range value overflow > 200 mm
ERROR_RANGEOFLOW   = const(15)

class VL6180X :

    def __init__  (self, I2C_adr, i2c = None, **kwargs) :

        if (I2C_adr < 0x00 or I2C_adr > 0x7F): # Vérifier que les adresses I2C sont dans {0x00, 0x7F}
            raise ValueError(
                'Unexpected mode value {0}.'.format(I2C_adr))
        self.I2C_adr = I2C_adr

        if i2c is None:
            raise ValueError('An I2C object is required.')
        self.i2c = i2c

        i2c.writeto_mem(self.I2C_adr, 0x0207, 0x01, addrsize=16) # WriteByte(0x0207, 0x01)
        i2c.writeto_mem(self.I2C_adr, 0x0208, 0x01, addrsize=16) # WriteByte(0x0208, 0x01)
        i2c.writeto_mem(self.I2C_adr, 0x0096, 0x00, addrsize=16) # WriteByte(0x0096, 0x00)
        i2c.writeto_mem(self.I2C_adr, 0x0097, 0xfd, addrsize=16) # WriteByte(0x0097, 0xfd)
        i2c.writeto_mem(self.I2C_adr, 0x00e3, 0x00, addrsize=16) # WriteByte(0x00e3, 0x00)
        i2c.writeto_mem(self.I2C_adr, 0x00e4, 0x04, addrsize=16) # WriteByte(0x00e4, 0x04)
        i2c.writeto_mem(self.I2C_adr, 0x00e5, 0x02, addrsize=16) # WriteByte(0x00e5, 0x02)
        i2c.writeto_mem(self.I2C_adr, 0x00e6, 0x01, addrsize=16) # WriteByte(0x00e6, 0x01)
        i2c.writeto_mem(self.I2C_adr, 0x00e7, 0x03, addrsize=16) # WriteByte(0x00e7, 0x03)
        i2c.writeto_mem(self.I2C_adr, 0x00f5, 0x02, addrsize=16) # WriteByte(0x00f5, 0x02)
        i2c.writeto_mem(self.I2C_adr, 0x00d9, 0x05, addrsize=16) # WriteByte(0x00d9, 0x05)
        i2c.writeto_mem(self.I2C_adr, 0x00db, 0xce, addrsize=16) # WriteByte(0x00db, 0xce)
        i2c.writeto_mem(self.I2C_adr, 0x00dc, 0x03, addrsize=16) # WriteByte(0x00dc, 0x03)
        i2c.writeto_mem(self.I2C_adr, 0x00dd, 0xf8, addrsize=16) # WriteByte(0x00dd, 0xf8)
        i2c.writeto_mem(self.I2C_adr, 0x009f, 0x00, addrsize=16) # WriteByte(0x009f, 0x00)
        i2c.writeto_mem(self.I2C_adr, 0x00a3, 0x3c, addrsize=16) # WriteByte(0x00a3, 0x3c)
        i2c.writeto_mem(self.I2C_adr, 0x00b7, 0x00, addrsize=16) # WriteByte(0x00b7, 0x00)
        i2c.writeto_mem(self.I2C_adr, 0x00bb, 0x3c, addrsize=16) # WriteByte(0x00bb, 0x3c)
        i2c.writeto_mem(self.I2C_adr, 0x00b2, 0x09, addrsize=16) # WriteByte(0x00b2, 0x09)
        i2c.writeto_mem(self.I2C_adr, 0x00ca, 0x09, addrsize=16) # WriteByte(0x00ca, 0x09)
        i2c.writeto_mem(self.I2C_adr, 0x0198, 0x01, addrsize=16) # WriteByte(0x0198, 0x01)
        i2c.writeto_mem(self.I2C_adr, 0x01b0, 0x17, addrsize=16) # WriteByte(0x01b0, 0x17)
        i2c.writeto_mem(self.I2C_adr, 0x01ad, 0x00, addrsize=16) # WriteByte(0x01ad, 0x00)
        i2c.writeto_mem(self.I2C_adr, 0x00ff, 0x05, addrsize=16) # WriteByte(0x00ff, 0x05)
        i2c.writeto_mem(self.I2C_adr, 0x0100, 0x05, addrsize=16) # WriteByte(0x0100, 0x05)
        i2c.writeto_mem(self.I2C_adr, 0x0199, 0x05, addrsize=16) # WriteByte(0x0199, 0x05)
        i2c.writeto_mem(self.I2C_adr, 0x01a6, 0x1b, addrsize=16) # WriteByte(0x01a6, 0x1b)
        i2c.writeto_mem(self.I2C_adr, 0x01ac, 0x3e, addrsize=16) # WriteByte(0x01ac, 0x3e)
        i2c.writeto_mem(self.I2C_adr, 0x01a7, 0x1f, addrsize=16) # WriteByte(0x01a7, 0x1f)
        i2c.writeto_mem(self.I2C_adr, 0x0030, 0x00, addrsize=16) # WriteByte(0x0030, 0x00)

        # Recommandé : registres publics
        #i2c.writeto_mem(self.I2C_adr, 0x0011, 0x10, addrsize=16)         # WriteByte(0x0011, 0x10) Prise en compte INT via GPIO1; Si état bas : mesure en cours
                                                                         # Valeur par défaut : 0x20 (cf. p 55 doc. ST)
        i2c.writeto_mem(self.I2C_adr, 0x010a, 0x30, addrsize=16)         # WriteByte(0x010a, 0x30) Averaging sample period : compromis entre bruit et temps d'exécution
                                                                         # Valeur par défaut : 0x30 (cf. p 75 doc. ST)
        i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSALS_ANALOGUE_GAIN, ALS_GAIN_1, addrsize=16)   # WriteByte(0x003f, 0x46) Valeur du gain pour mesure de lumière : 1
                                                                         # Valeur par défaut : 0x46 (cf. p 67 doc. ST)
        i2c.writeto_mem(self.I2C_adr, 0x0031, 0xFF, addrsize=16)         # WriteByte(0x0031, 0xFF) // Calibration toutes les 0xFF mesures
                                                                         # Valeur par défaut : 0x00 (cf. p 64 doc. ST)
        i2c.writeto_mem(self.I2C_adr, 0x0040, 0x63, addrsize=16)         # WriteByte(0x0040, 0x63) // Temps d'intégration pour mesure de lumière : 100ms
                                                                         # Valeur par défaut : 0x00 (cf. p 67 doc. ST)
                                                                         # valeur recommandée : 100ms soit 0x63
        i2c.writeto_mem(self.I2C_adr, 0x002e, 0x01, addrsize=16)         # WriteByte(0x002e, 0x01) // Effectuer une seule fois la calibration du capteur en température
                                                                         # Valeur par défaut : 0x00 (cf. p 64 doc. ST)

    # Options : registres publics
    #i2c.writeto_mem(self.I2C_adr, 0x001b, 0x09, addrsize=16)         # WriteByte(0x001b, 0x09) // Set default ranging inter-measurement : 100ms
                                                                     # Valeur par défaut : 0xFF (cf. p 60 doc. ST)

    #i2c.writeto_mem(self.I2C_adr, 0x003e, 0x31, addrsize=16)         # WriteByte(0x003e, 0x31) // Set default ALS inter-measurement period : 500ms
                                                                      # Valeur par défaut : 0xFF (cf. p 66 doc. ST)

        i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, 0x24, addrsize=16)  # WriteByte(0x0014, 0x24) // Configures interrupt on ‘New Sample' : Nouvelle mesures de distance ou luminosité dispo
                                                                                           # Valeur par défaut : 0x00 (cf. p 57 doc. ST)
#---------------------------------------------------------------------------
    # Permet de modifier l'adresse I2C d'un capteur VL6180X lorsque plusieurs sont utilisés simultanèment
    # Adr par défaut : 0x29 (soit 41)
    def Modif_Adr_I2C (self, CE_pin, new_i2c_adr, old_i2c_adr) :
        CE_pin.value(1) # Enable capteur
        time.sleep(0.0015) # Attendre 1.5ms
        # Lecture du contenu du registre d'@ 0x16 : Ok
        Registre_data = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 1, addrsize=16)
        if Registre_data[0] == 0x01 : # La procédure de reset hard est achevée et le capteur est fonctionnel
            # Mise à 0 du contenu du registre d'@ 0x16 : Ok
            self.i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0, addrsize=16)
            # Modifier et mettre à jour l'@I2C du capteur
            if old_i2c_adr != new_i2c_adr :
                self.i2c.writeto_mem(self.I2C_adr, VL6180X_REG_I2C_SLAVE_DEVICE_ADDRESS, new_i2c_adr, addrsize=16)
                self.I2C_adr = new_i2c_adr
#---------------------------------------------------------------------------
    # Effectuer une mesure de distance en mode single shot
    def range_mesure (self) :
        self.i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSRANGE_START, 0x01, addrsize=16) # Mode single shot et début de la mesure
        time.sleep(0.001)
        # Attendre que la mesure soit complète
        intr_status_reg = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO, 1, addrsize=16)
        range_status = intr_status_reg[0] & 0x04
        while (range_status != 0x04) : # Tant que la mesure n'est pas terminée, attendre
            intr_status_reg = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO, 1, addrsize=16)
            time.sleep (0.001)
            range_status = intr_status_reg[0] & 0x04
        # Lire le contenu du registre contenant la valeur mesurèe en mm
        distance = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_RESULT_RANGE_VAL, 1, addrsize=16)
        # Remettre le registre de statut des interruptions
        self.i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07, addrsize=16)
        return distance[0]
#---------------------------------------------------------------------------
    # Effectuer une mesure de luminosité ambiante en mode single shot
    def ambiant_light_mesure (self) :
        self.i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSALS_START, 0x01, addrsize=16) # Débuter la mesure
        time.sleep(0.001)
        # Attendre que la mesure soit complète
        intr_status_reg = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO, 1, addrsize=16)
        time.sleep(0.001)
        ambiant_light_status = intr_status_reg[0] & 0x20
        while (ambiant_light_status != 0x20) : # Tant que la mesure n'est pas terminée, attendre
            intr_status_reg = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO, 1, addrsize=16)
            time.sleep(0.001)
            ambiant_light_status = intr_status_reg[0] & 0x20
        # Lire le registre contenant la mesure de luminosité ambiant_light_mesure
        rslt_als = self.i2c.readfrom_mem(self.I2C_adr, VL6180X_REG_RESULT_ALS_VAL, 2, addrsize=16)
        time.sleep(0.001)
        # Remettre le registre de statut des interruptions
        self.i2c.writeto_mem(self.I2C_adr, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07, addrsize=16)
        time.sleep(0.001)
        # Calcul de la luminosité ambiante en lux; temps d'intégration de 100ms; cf doc ST p 38
        lux = 0.32 * (rslt_als[0] << 8 | rslt_als[1]) # ALS_GAIN = 1 : init par défaut
        return lux # ALS_GAIN = 1 : init par défaut
