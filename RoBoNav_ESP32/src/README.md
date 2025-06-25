# RoboNav
 
## Projet RoBoNav 

Développé par Gauthier AILLERET, Pierre-Louis BURGUET, Nicolas FERRY, Enora FREMY, Marie LOUVET (ICAM de Nantes)
et Jean FRUITET (ARBL)

Contact : jean.fruitet@free.fr

Ce projet de Bouée autonome avec ancrage par GPS destinée à la VRC (Voile Radio Commandée) 
a été lancé en février 2023 à l'initiative de Jean FRUITET avec le soutien de l'ARBL (Association Radiomodéliste des Bords de Loire).
Deux équipes projets successives d'élèves de 4ème année de l'école d'ingénieurs ICAM de Nantes 
l'ont mené à bien sous la tutelle de Nicolas FERRY, enseignant-chercheur.

Les tests menés en juillet 2024 ont validé les concepts, le modèle de bouée et les logiciels
de pilotage.

Ce projet n'est pas achevé, des améliorations seront développées graduellement en sources libres.

Si vous êtes intéressé contactez-moi.


## Architecture



Voir la documentation dédiée à la partie électronique du projet

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

##### Structure

(A terminer)