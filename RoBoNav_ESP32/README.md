# RoboNav
 
## Projet RoBoNav 

Développé par Gauthier AILLERET, Pierre-Louis BURGUET, Nicolas FERRY, Enora FREMY, Marie LOUVET, Agathe DAUDENTHUN, Thed Primael KAMGA KAPTOUOM, (ICAM de Nantes)
Mathis ANIZON, Quentin PARIS, ICAM de Vannes,
et Jean FRUITET (ARBL)

Contact : jean.fruitet@free.fr

Ce projet de Bouée autonome avec ancrage par GPS destinée à la VRC (Voile Radio Commandée) 
a été lancé en février 2023 à l'initiative de Jean FRUITET avec le soutien de l'ARBL (Association Radiomodéliste des Bords de Loire).
Des équipes projets successives d'élèves de 4ème année de l'école d'ingénieurs ICAM de Nantes 
l'ont mené à bien sous la tutelle de Nicolas FERRY, enseignant-chercheur.

A partir de septembre 2025, Mathis ANIZON et Quentin PARIS, élèves de 5ème année en alternance à l'ICAM de Vannes ont contribué à sa finalisation,
sous la direction de Laurane GILETTE.

En 2025 le positionnement GPS a été affiné, en testant plusieurs modèles de GPS d'entrée de gamme.

Finalement nous nous sommes orientés vers l'utilisation de GPS RTK pour obtenir une précision décimétrique
permettant de distinguer les déplacements dus à la dérive du vent et/ou du courant des errement imputables à l'impressision des données GPS.

Ce projet n'est pas achevé, des améliorations seront développées graduellement en sources libres.

Si vous êtes intéressé contactez-moi.

JF.

## Architecture

* Bouée + carte contrôleur ICAM/ARBL + WiFi + GPS RTK + Compas + 2 turbines + ESC + batteries LiPo
* Application smartphone communiquant par broadcast avec les bouées
* Base GPS RTK + GPS RTK rover sur les bouées

Voir la documentation disponible dans le dossier  ../Documentation

### Pilotage
Nous utilisons une radio commande Rodiomaster (compatible FrSky)
et/ou une applet sur smartphone développée avec MIT App Inventor 2.

### Bouées
Le bouées ont une base flottante en EPP et une couronne mobile en PE.

La motorisation se fait avec deux turbines brushless connectées à des ESC réversibles alimentées en 12 volts pas des batteries LiPo 3S ou une batterie au plomb 12 volts de moto.

### Communication et positionnement
Le dispositif de pilotage et de positionnement est composé d'une partie électronique et de logiciels dédiés.

#### Contrôleur de navigation
Nous avons développé une contrôleur de navigation basé sur micro contrôleur ESP32 WROOM WiFi.

Un circuit imprimé (PCB) spécifique a été conçu pour ce projet.

Le chip ESP32 gère un récepteur Radiomaster, une connexion WiFi, une antenne GPS, une boussole et des accéléromètres.

#### Logiciels

##### Microcontrôleur
La gestion de la carte de navigation est développée en CPP (.ino) sous  l'IDE Android Studio
L'ensemble des sources est dans le dossier ./src

##### Réseau WiFi
L'établissement d'u réseau wiFi pour communique par brodcast avec les bouées à partir d'un smartphone ou d'un PC nécessite
un SSID et un mode passe.
Le fichier ./src/RoBoNav_config_WiFi.h doit être modifié en conséquence

##### Interface de pilotage

Une application pour smartphone Android a été développées sous MIT App Inventor (https://appinventor.mit.edu/)

