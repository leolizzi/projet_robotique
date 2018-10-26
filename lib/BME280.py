# Gestion du capteur BME280
# Mesure de température, humidité et pression atmosphèrique
# Température : en °C
# Humidité relative : en % sur [0.0; 100.0]"
# Pression atmosphérique : en hPa

# Ressources carte Wipy 3.0
    # Bus I2C
        # P9 : SDA
        # P10 : SCL

from micropython import const
from machine import I2C
import time

#------------------------------------------------------------------------------------------------------------------
# Constantes symboliques pour BME280
BME280_CORRECTION_PRESSION = const(500) # en Pa

BME280_I2C_ADR = const(0x77) # Adresse I2C du capteur BME280 (119 en décimal pour le shield Adafruit BME280)
                             # Adresse I2C du capteur BME280 (118 en décimal pour le shield Grove BME280)
BME280_ID_CHIP  = const(0x60) # Identifiant du capteur BME280 (96 en décimal)

# Définition de la taille de champs de données
TAILLE_BUFFER = const(26)
BME280_TEMP_PRESS_CALIB_DATA_LEN = TAILLE_BUFFER # Paramétres de calibration T1...T3 + P1...P7 + H1, soit 26 octets de type uint8_t
BME280_HUMIDITY_CALIB_DATA_LEN = const(7)        # Paramétres de calibration H2...H6, soit 7 octets
BME280_P_T_H_DATA_LEN = const(8)                 # Registres de température, pression et humidité : 8 octets

# Name Register Address
BME280_CHIP_ID_ADDR = const(0xD0)   # Adr du registre Id
BME280_RESET_ADDR = const(0xE0)     # Adr du registre de reset
BME280_CTRL_HUM_ADDR = const(0xF2)  # Adr du registre de contrôle de la mesure d'humidité
BME280_CTRL_STATUT = const(0xF3)    # Adr du registre de statut du capteur
BME280_CTRL_MEAS_ADDR = const(0xF4) # Adr du registre de contrôle des mesures
BME280_CONFIG_ADDR = const(0xF5)    # Adr du registre de config
BME280_DATA_ADDR = const(0xF7)      # Adr de base des données de température, pression et humidité non compensées

# Paramètres de calibration
BME280_TEMP_PRESS_CALIB_DATA_ADDR = const(0x88) # Adr de la zone des paramètres de calibration pour température et pression
BME280_HUMIDITY_CALIB_DATA_ADDR	= const(0xE1)  # Adr de la zone des paramètres de calibration pour humidité

# Modes de contrôle du capteur
BME280_SLEEP_MODE = const(0x00)    # Mode sommeil
BME280_FORCED_MODE = const(0x01)   # Mode forcé
BME280_NORMAL_MODE = const(0x03)   # Mode normal

# Valeur du facteur de sur échantillonnage
BME280_NO_OVERSAMPLING = const(0B000)
BME280_OVERSAMPLING_1X = const(0B001)
BME280_OVERSAMPLING_2X = const(0B010)
BME280_OVERSAMPLING_4X = const(0B011)
BME280_OVERSAMPLING_8X = const(0B0100)
BME280_OVERSAMPLING_16X = const(0B101)

# Durée d'inactivité du BME280
BME280_STANDBY_TIME_1_MS = const(0B000)
BME280_STANDBY_TIME_62_5_MS = const(0B001)
BME280_STANDBY_TIME_125_MS = const(0B0010)
BME280_STANDBY_TIME_250_MS = const(0B0011)
BME280_STANDBY_TIME_500_MS = const(0B0100)
BME280_STANDBY_TIME_1000_MS = const(0B0101)
BME280_STANDBY_TIME_10_MS = const(0B0110)
BME280_STANDBY_TIME_20_MS = const(0B0111)

# Sélection des coefficients du filtre IIR
BME280_FILTER_COEFF_OFF = const(0B000)
BME280_FILTER_COEFF_2 = const(0B001)
BME280_FILTER_COEFF_4 = const(0B010)
BME280_FILTER_COEFF_8 = const(0B011)
BME280_FILTER_COEFF_16 = const(0B0100)

class BME280 :
    def __init__  (self, osr_p, osr_t, osr_h, config_filter, standby_time, mode, I2C_adr, i2c = None, **kwargs) :
        if osr_p not in [BME280_OVERSAMPLING_1X, BME280_OVERSAMPLING_2X, BME280_OVERSAMPLING_4X,
                        BME280_OVERSAMPLING_8X, BME280_OVERSAMPLING_16X]:
            raise ValueError(
                'Unexpected osr_p value {0}.'.format(osr_p))
        self.osr_p = osr_p

        if osr_t not in [BME280_OVERSAMPLING_1X, BME280_OVERSAMPLING_2X, BME280_OVERSAMPLING_4X,
                        BME280_OVERSAMPLING_8X, BME280_OVERSAMPLING_16X]:
            raise ValueError(
                'Unexpected osr_t value {0}.'.format(osr_t))
        self.osr_t = osr_t

        if osr_h not in [BME280_OVERSAMPLING_1X, BME280_OVERSAMPLING_2X, BME280_OVERSAMPLING_4X,
                        BME280_OVERSAMPLING_8X, BME280_OVERSAMPLING_16X]:
            raise ValueError(
                'Unexpected osr_h value {0}.'.format(osr_h))
        self.osr_h = osr_h

        if config_filter not in [BME280_FILTER_COEFF_OFF, BME280_FILTER_COEFF_2, BME280_FILTER_COEFF_4,
                                BME280_FILTER_COEFF_8, BME280_FILTER_COEFF_16]:
            raise ValueError(
                'Unexpected config_filter value value {0}.'.format(config_filter))
        self.config_filter = config_filter

        if standby_time not in [BME280_STANDBY_TIME_1_MS, BME280_STANDBY_TIME_62_5_MS, BME280_STANDBY_TIME_125_MS,
                            BME280_STANDBY_TIME_250_MS, BME280_STANDBY_TIME_500_MS, BME280_STANDBY_TIME_1000_MS,
                            BME280_STANDBY_TIME_10_MS, BME280_STANDBY_TIME_20_MS]:
            raise ValueError(
                'Unexpected standby_time value {0}.'.format(standby_time))
        self.standby_time = standby_time

        if mode not in [BME280_SLEEP_MODE, BME280_FORCED_MODE, BME280_NORMAL_MODE ]:
            raise ValueError(
                'Unexpected mode value {0}.'.format(mode))
        self.mode = mode

        if I2C_adr not in [0x76, 0x77]:
            raise ValueError(
                'Unexpected mode value {0}.'.format(I2C_adr))
        self.I2C_adr = I2C_adr

        # Mise en forme contenu des registres de configuration
        reg_ctrl_hum = self.osr_h
        reg_config = (self.config_filter << 2) | (self.standby_time << 5)
        reg_ctrl_meas = self.mode | (self.osr_p << 2) | (self.osr_t << 5)

        if i2c is None:
            raise ValueError('An I2C object is required.')
        self.i2c = i2c

        # Ecriture des registres de configuration
        i2c.writeto_mem(self.I2C_adr, BME280_CTRL_HUM_ADDR, reg_ctrl_hum)
        time.sleep (0.004)
        i2c.writeto_mem(self.I2C_adr, BME280_CONFIG_ADDR, reg_config)
        time.sleep (0.004)
        i2c.writeto_mem(self.I2C_adr, BME280_CTRL_MEAS_ADDR, reg_ctrl_meas)
        time.sleep (0.004)
    #---------------------------------------------------------------------------
    # Lire la valeurs des registres des paramétres de calibration à partir du registre d'@ 0x88, soient 26 octets
    # Lecture de la zone mémoire contenant la valeurs des paramètres de calibration dig_T1 to dig_T3, dig_P1 to dig_P7 et dig_H1
    # Extraction des paramètres de calibration : validée le 09.03.2018
    def Calibration_Param_Load (self) :
        Registre_buffer = self.i2c.readfrom_mem(self.I2C_adr, BME280_TEMP_PRESS_CALIB_DATA_ADDR, BME280_TEMP_PRESS_CALIB_DATA_LEN)
        time.sleep (0.004)
        # Parser le buffer afin de calculer la valeur des paramètres de calibration
        self.dig_T1 = Registre_buffer[1] << 8 | Registre_buffer[0] #entier non signé
        self.dig_T2 = Registre_buffer[3] << 8 | Registre_buffer[2] #entier signé
        self.dig_T2 = self.Conversion_type_signedint16 (self.dig_T2)
        self.dig_T3 = Registre_buffer[5] << 8 | Registre_buffer[4] #entier signé
        self.dig_T3 = self.Conversion_type_signedint16 (self.dig_T3)
        self.dig_P1 = Registre_buffer[7] << 8 | Registre_buffer[6] #entier non signé
        self.dig_P2 = (Registre_buffer[9] << 8 | Registre_buffer[8]) #entier signé
        self.dig_P2 = self.Conversion_type_signedint16 (self.dig_P2)
        self.dig_P3 = Registre_buffer[11] << 8 | Registre_buffer[10] #entier signé
        self.dig_P3 = self.Conversion_type_signedint16 (self.dig_P3)
        self.dig_P4 = Registre_buffer[13] << 8 | Registre_buffer[12] #entier signé
        self.dig_P4 = self.Conversion_type_signedint16 (self.dig_P4)
        self.dig_P5 = Registre_buffer[15] << 8 | Registre_buffer[14] #entier signé
        self.dig_P5 = self.Conversion_type_signedint16 (self.dig_P5)
        self.dig_P6 = Registre_buffer[17] << 8 | Registre_buffer[16] #entier signé
        self.dig_P6 = self.Conversion_type_signedint16 (self.dig_P6)
        self.dig_P7 = Registre_buffer[19] << 8 | Registre_buffer[18] #entier signé
        self.dig_P7 = self.Conversion_type_signedint16 (self.dig_P7)
        self.dig_P8 = Registre_buffer[21] << 8 | Registre_buffer[20] #entier signé
        self.dig_P8 = self.Conversion_type_signedint16 (self.dig_P8)
        self.dig_P9 = Registre_buffer[23] << 8 | Registre_buffer[22] #entier signé
        self.dig_H1 = Registre_buffer[25] #entier non signé
    # Lire la valeurs des registres des paramétres de calibration à partir du registre d'@ 0xE1, soient 7 octets
    # Lecture de la zone mémoire contenant la valeurs des paramètres de calibration dig_H2 to dig_H6
        Registre_buffer = self.i2c.readfrom_mem(self.I2C_adr, BME280_HUMIDITY_CALIB_DATA_ADDR, BME280_HUMIDITY_CALIB_DATA_LEN)
        time.sleep (0.004)
        # Parser le buffer afin de calculer la valeur des paramètres de calibration
        self.dig_H2 = Registre_buffer[1] << 8 | Registre_buffer[0] #entier signé
        self.dig_H2 = self.Conversion_type_signedint16 (self.dig_H2)
        self.dig_H3 = Registre_buffer[2] #entier non signé
        self.dig_H4 = Registre_buffer[3] << 4 | Registre_buffer[4] #entier signé
        self.dig_H4 = self.Conversion_type_signedint16 (self.dig_H4)
        self.dig_H5 = Registre_buffer[4] >> 4 | Registre_buffer[5] << 4 #entier signé
        self.dig_H5 = self.Conversion_type_signedint16 (self.dig_H5)
        self.dig_H6 = Registre_buffer[6] #entier signé
        self.dig_H6 = self.Conversion_type_signedint8 (self.dig_H6)
    #---------------------------------------------------------------------------
    # Acquisition de la valeur de température non compensée
    def U_temperature (self) :
        # Lecture de la valeur des registres de température, pression et humidité
        self.BME280_Data = self.i2c.readfrom_mem(self.I2C_adr, BME280_DATA_ADDR, BME280_P_T_H_DATA_LEN)
        time.sleep (0.004)
        # Calcul de la valeurs non compensée de température
        u_t = (self.BME280_Data[3] << 12) | (self.BME280_Data[4] << 4) | (self.BME280_Data[5] >> 4)
        return u_t
    #---------------------------------------------------------------------------
    # Valeur de pression non compensée
    def U_pression (self) :
        # Calcul de la valeur non compensée de pression
        u_p = (self.BME280_Data[0] << 12) | ( self.BME280_Data[1] << 4) | (self.BME280_Data[2] >> 4)
        return u_p
    #---------------------------------------------------------------------------
    # Valeur de taux d'humidité non compensée
    def U_humidity (self) :
        # Calcul de la valeur non compensée de pression
        u_h = self.BME280_Data[6] << 8 | self.BME280_Data[7]
        return u_h
    #---------------------------------------------------------------------------
    def read_temp(self) : # Calculer la valeur compensée de la température en °C
        var1 = 0.0
        var2 = 0.0
        temperature = 0.0
        temperature_min = 0.0
        temperature_max = 85.0
        self.t_fine = 0.0

        u_data_temp = self.U_temperature()
        var1 = float (u_data_temp) / 16384.0 - float (self.dig_T1) / 1024.0
        var1 = var1 * float (self.dig_T2)
        var2 = (float (u_data_temp) / 131072.0) - (float (self.dig_T1) / 8192.0)
        var2 = var2 * var2 * (float (self.dig_T3))
        self.t_fine = (var1 + var2)
        temperature = (var1 + var2) / 5120.0
        if (temperature < temperature_min) :
            temperature = temperature_min
        elif (temperature > temperature_max) :
            temperature = temperature_max
        return temperature
    #---------------------------------------------------------------------------
    def read_pression (self) : # Calculer la valeur compensée de la pression
    # Nécessite de calculer au préalable t_fine
        var1 = 0.0
        var2 = 0.0
        var3 = 0.0
        p = 0.0
        pression_min = 30000.0 # En Pa
        pression_max = 110000.0 # En Pa

        u_data_press = self.U_pression()
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * float (self.dig_P6) / 32768.0
        var2 = var2 + var1 * float (self.dig_P5) * 2.0
        var2 = (var2 / 4.0) + (float (self.dig_P4) * 65536.0)
        var3 = float (self.dig_P3) * var1 * var1 / 524288.0
        var1 = (var3 + float (self.dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * float (self.dig_P1)
        # Eviter une exception suite à division par 0
        if (var1) :
            p = 1048576.0 - float (u_data_press)
            p = (p - (var2 / 4096.0)) * 6250.0 / var1
            var1 = float(self.dig_P9) * p * p / 2147483648.0
            var2 = p * float(self.dig_P8) / 32768.0
            p = (p + (var1 + var2 + float (self.dig_P7)) / 16.0)
            p = (p + BME280_CORRECTION_PRESSION)
            if (p < pression_min) :
                p = pression_min
            elif (p > pression_max) :
                p = pression_max
        else :
            p = 0.0
        return p / 100.0 # Pression en hPa
    #---------------------------------------------------------------------------
    def read_humidity (self) : # Calculer la valeur compensée de l'humidité
        humidity_min = 0.0
        humidity_max = 100.0
        var1 = 0.0
        var2 = 0.0
        var3 = 0.0
        var4 = 0.0
        var5 = 0.0
        var6 = 0.0

        u_data_hum = self.U_humidity ()
        var1 = float (self.t_fine) - 76800.0
        var2 = (float (self.dig_H4) * 64.0) + ((float (self.dig_H5) / 16384.0) * var1)
        var3 = u_data_hum - var2
        var4 = float (self.dig_H2) / 65536.0
        var5 = (1.0 + ((float (self.dig_H3)) / 67108864.0) * var1)
        var6 = 1.0 + ((float (self.dig_H6)) / 67108864.0) * var1 * var5
        var6 = var3 * var4 * (var5 * var6)
        humidity = var6 * (1.0 - float (self.dig_H1) * var6 / 524288.0)
        if (humidity > humidity_max) :
            humidity = humidity_max
        elif (humidity < humidity_min) :
            humidity = humidity_min
        return humidity
    #---------------------------------------------------------------------------
    # Fonctions de conversion de types entiers non signés en représentation signée
    #---------------------------------------------------------------------------
    @staticmethod
    def Conversion_type_signedint16 (n) : # forcer la représentation en int16_t d'un entier sur 2 octets
        if n >= (0x8000)  : return n - (0xFFFF) - 1
        else :
            return n
        #---------------------------------------------------------------------------
    @staticmethod
    def Conversion_type_signedint8 (n) : # forcer la représentation en int8_t d'un entier sur 1 octet
        if n >=  (0x80)  : return n - (0xFF) - 1
        else :
            return n
    #---------------------------------------------------------------------------

#On définie le capteur
capteurTemp = BME280(BME280_OVERSAMPLING_16X,BME280_OVERSAMPLING_16X,BME280_OVERSAMPLING_16X,BME280_FILTER_COEFF_2,BME280_STANDBY_TIME_125_MS,BME280_NORMAL_MODE,BME280_I2C_ADR,bus_i2c)

#On calibre le capteur
capteurTemp.Calibration_Param_Load()

def lireTemperature(capteurTemp):
    return capteurTemp.read_temp()

def lireHumidite(capteurTemp):
    return capteurTemp.read_humidity()

def lirePression(capteurTemp):
    #Nécessite d'avoir lu la température avant (normalement)
    return capteurTemp.read_pression()
