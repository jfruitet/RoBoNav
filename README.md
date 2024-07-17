# RoboNav
 
## Projet RoBoNav 

Développé par Gauthier AILLERET, Pierre-Louis BURGUET, Nicolas FERRY, Enora FREMY, Marie LOUVET (ICAM de Nantes)
et Jean FRUITET (ARBL)

Contact : jean.fruitet@free.fr
GitHub: 

Ce projet de Bouée autonome avec ancrage par GPS destinée à la VRC (Voile Radio Commandée) 
a été lancé en février 2023 à l'initiative de Jean FRUITET avec le soutien de l'ARBL (Association Radiomodéliste des Bords de Loire).

Deux équipes projets successives d'élèves de 4ème année de l'école d'ingénieurs ICAM de Nantes 
l'ont mené à bien sous la tutelle de Nicolas FERRY, enseignant-chercheur.

Les tests effectués en juillet 2024 ont validé les concepts, le modèle de bouée et les logiciels de pilotage.

Ce projet n'est pas achevé, des améliorations seront développées graduellement en sources libres.

Si vous êtes intéressé contactez-moi.


## Architecture

Voir la documentation détaillée dédiée au projet.

### Pilotage
Nous utilisons une radio commande Rodiomaster (compatible FrSky)
et/ou une applet sur smartphone développée avec MIT App Inventor 2.

### Bouées
Le bouées ont une base flottante circulaire en EPP (polypropylène expansé) et une couronne mobile en polystyrène expansé.

La motorisation se fait avec deux turbines brushless connectées à des ESC réversibles alimentées en 12 volts 
par des batteries LiPo 3S ou par une batterie de moto au plomb de 12 volts.

### Communication et positionnement
Le dispositif de pilotage et de positionnement est composé d'une partie électronique et de logiciels dédiés.

#### Contrôleur de navigation
Nous avons développé une contrôleur de navigation basé sur micro contrôleur ESP32.

Un circuit imprimé (PCB) spécifique a été conçu pour ce projet.

Le chip ESP32 gère un récepteur Radiomaster, une connexion WiFi, une antenne GPS, une boussole et des accéléromètres.

#### Logiciels
Trois types de logiciels ont été développés :
    Code C (.ino) pour le micro contrôleur.
    Code MIT App Inventor pour le pilotage (émulation d'une radiocommande)
    Code PHP pour le serveur de sites de navigation et le placement des bouées de régate.
    
##### Microcontrôleur
La gestion de la carte de navigation est développée en C (.h et .ino) sous  Android Studio

Voir le dossier RoBoNav_ESP32

##### Pilotage avec un smartphone
Une applet pour smartphone Android émule une radio commande. 

Voir le dossier RoBoNav_smartphone

#### Serveur cartographique de placement des Bouées
Pour chaque plan d'eau de la région nantaise j'ai développé une application Web permettant de repérer les positions GPS à assigner 
à chaque bouées d'un parcours de régate, en fonction de la direction du vent.

Le fichier JSON des positions GPS est ensuite chargé par l'application smartphone et les positions respectives 
envoyées aux bouée du parcours.

(Réalisation en cours)
Voir le GitHub https://github.com/jfruitet/placerbouees

## ETAT D'AVANCEMENT DU PROJET

### REALISE
* *FAIT* : Bouée RoBoNav v 1.0, motorisation, alimentation
* *FAIT* : Carte électronique et circuit imprimé pour chip ESP32 WROOM
* *FAIT* : Pilotage de la bouée RoBoNav avec une radiocommande Radiomaster 
* *FAIT* : Configuration et émission / réception WiFi (protocole UDP)
* *FAIT* : Pilotage de la bouée RoBoNav avec une applet Android (communication par WiFi) - Logiciel 'Telecommande.aia' développé sous MIT App Inventor 2.
* *FAIT* : Tester la configuration et la réception GPS
* *FAIT* : Implanter le support de la boussole
* *FAIT* : Tester le maintien à une position GPS (programmer différents algorithmes de positionnement dont un PID)
* *FAIT* : Tester le RTH
* *FAIT* : Tester la bascule depuis la radiocommande vers le smartphone et vice-versa
* *FAIT* : Serveur de fichiers de positionnement des bouées en fonction du vent
* *FAIT* : Créer un GitHub RoBoNav

### A FAIRE
* Programmer le fail safe
* Rassembler les paramètres de configuration dans un fichier de setup
* Implanter le chargement / assignation de positions GPS par Smartphone / WiFi
* Rassembler la documentation
* Traduire la documentation en anglais 
* Créer une page Facebook RoboNav
