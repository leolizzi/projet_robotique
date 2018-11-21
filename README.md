# Projet robotique PeiP1 Polytech Tours
Projet réalisé dans le cadre du parcours PeiP de Polytech Tours.

Liste du matériel :

- Pycom 3
- Wipy
- Capteur BME280 (température, humidité et pression)
- Capteur VL6180X (distance et luminosité)
- DRV8833 (driver moteurs)
- Moteur FIT0483

Le but est de créer un petit robot autonome à deux roues qui devra évoluer dans un espace de 4x3m. 
Il devra être capable de se déplacer dans l'intégralité de l'espace et récolter des informations sur son passage
(position (x,y), température, luminosité, distance à l'obstacle...) et les stocker dans une carte SD sous forme d'un fichier SVG.

Le fichier SVG est organisé de la manière suivante :

- Un numéro unique de ligne (pid)
- La coordonnée X du robot (x)
- La coordonnée Y du robot (y)
- Le temps en seconde écoulé depuis le départ du robot (t)
- La distance à l’obstacle (d)
- L’orientation du robot (a)
- La température (c)
- La luminosité (l)

