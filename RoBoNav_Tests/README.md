# Tests de la consommation et de dérive due au vent

Rédigé par JF le 27/05/2025

Suite aux difficultés dues à l'incertitude sur le positionnement des bouées avec des GPS d'entrée de gamme (Ublox M8N) et dans l'attente de GPS RTK,  je vous propose de mettre en place deux séries de tests à réaliser sur le plan d'eau du Plessis. 

## Test de consommation

Il s’agit de déterminer l’autonomie des batteries utilisées (2 LiPo 3S en parallèle)
On mesurera la tension avant et après des séries de déplacement sur une distance matérialisée par deux bouées ancrées à 10 mètres d’une de l’autre.

### Pré-requis

* Disposer d’une bouée RoBoNav apte à naviguer, pilotée par télécommande

* Disposer d’un anémomètre et d’un chronomètre ;

* Disposer de deux bouées ancrées (fournies par l’ARBL) ;

* Réalisation : Programmer une application sur PC ou sur smartphone récupérant en temps réel les données du test par WiFi :
  
  * la tension disponible sur la (les) batterie()s en début et fin de test ;
  
  * la puissance moyenne appliqués aux moteurs (en pourcentage ou en degré) pour un déplacement en ligne droite ;
  
  * la direction (relevée au compas) du déplacement ;
  
  * la durée du déplacement (qui peut être aussi chronométrée depuis la rive)

### Protocole

* Mesurer la force du vent avec un anémomètre.
* Matérialiser une distance de 20 mètres avec deux bouées ancrées, l’une au vent et l'autre sous le vent. 
* Réaliser plusieurs séries de tests pour mesurer :     
    Temps de déplacement sur 20 mètres face au vent → Vitesse moyenne
    Temps de déplacement sur 20 mètres dans le sens du vent → Vitesse moyenne 
    Consommation électrique à puissance faible (25%), moyenne (33%), forte (50%), très forte (100%)
    Relevé des points GPS (même imparfaits) et direction (au compas) entre le début et la fin de chaque test.

## Test de dérive

Il s’agit de déterminer la durée de dérive due au vent sur une distance matérialisée par deux bouées ancrées à 20 mètres d’une de l’autre dans le lit du vent.

On positionne la bouée RoBoNav au niveau de la bouée ancrée, moteurs arrêtés et on mesure le temps nécessaire pour qu’elle dérive jusqu’à la bouée aval.

On mesure alors le temps nécessaire pour la ramener à la bouée amont avec une puissance moteur de 50% de puissance.

On répète ces mesures pour une série de 10 parcours.

L’idéal serait de pouvoir effectuer ces mesures avec des conditions météos variées, par temps calme, vent moyen (10 km/h) et vent fort (20 km/h, 30km/h)...

## Réalisations

Dans le temps imparti les test n'ont pu être menés à bien. Agathe D. a toutefois pu réaliser en Python l'interface de l'application RoBoNav-Test.py destinée à récolter les data. 
