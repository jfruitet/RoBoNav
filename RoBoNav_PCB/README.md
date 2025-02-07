# Projet RoBoNav de bouées de régate VRC autonomes.

## PCB / Circuit imprimé ESP32-WROOM - ICAM / ARBM

Réalisation : Nicolas Ferry et Thierry Giorgetti ICAM de Nantes - Avril 2024

Contact jean.fruitet@free.fr

### Introduction
Plutôt que d'utiliser une carte contrôleur de vol toute faite de type **PixHawk** ou **Revolution**,
il nous  a semblé plus judicieux des dessiner notre propre carte à base de micro contrôleur ESP32-WROOM.

D'une part pour des questions de coût, d'autre part pour amener nos étudiants impliqués dans ce projet 
à se confronter à une réalisation "from scratch" avec tout le processus de design, routage, fabrication, 
implantation des composants et tests.

Nous disposons désormais d'une carte de navigation correspondant exactement à nos besoins pour un coût modique.

Dans le cadre de l'accord de partenariat entre l'ICAM et l'ARBL, l'ensemble des sources et des schémas 
sont disponibles sous licence MIT ce qui devrait garantir leur reprise (et amélioration...)
par tout utilisateur averti.

###   Répertoire : RoBoNav\RoBoNav_PSB

```
Mode                 Last WriteTime        Length Name                                                                 
----                 -------------         ------ ----                                                                 

Logos imprimés sur le PCB                                                 
-a----        18/07/2024     10:39          23022 ARBL_Logo.bmp 
-a----        18/07/2024     10:39          19374 ICAMLogo.bmp 

Schéma de la carte de navigation (PCB)
-a----        18/07/2024     10:39          69917 Schematic_Board_v0.7.png   

Fichier MicroSim PCBoard Status of Autorouting Run              
-a----        18/07/2024     10:39           2253 RobotNav.sts  

Routage du circuit imprimé                                                       
-a----        18/07/2024     10:39          29353 bestsave.rte 

Brochage du microcontrôleur ESP32                                                        
-a----        18/07/2024     10:39         254073 ESP32-DEV-KIT-DevKitC-v4-pinout-mischianti.jpg                       
-a----        18/07/2024     10:39         671462 esp32-devkitC-v4-pinout.png                                          
                                                                
Fichier de projet *Proteus Design Suite*                                                                
-a----        18/07/2024     10:39            132 RobotNav.DO                                                          
-a----        18/07/2024     10:39          29922 RobotNav.EDF                                                         
-a----        18/07/2024     10:39            209 RobotNav.log                                                         
-a----        18/07/2024     10:39          99568 RobotNav.pdsprj                                                      
-a----        18/07/2024     10:39           2073 RobotNav.pdsprj.ICAMAD.thierry.giorgetti.workspace                   
-a----        18/07/2024     10:39           1846 RobotNav.pdsprj.SERV-NICO-FER.Administrateur.workspace                                                                     
-a----        18/07/2024     10:39         104413 RobotNav7.pdsprj                                                     
-a----        18/07/2024     10:39           1840 RobotNav7.pdsprj.SERV-NICO-FER.Administrateur.workspace       


Schématique PCB de l'ESP32-WROOM à importer dans  *Proteus Design Suite*                                                     
-a----        18/07/2024     10:39        2286741 RobotNav.zip     
                                                    RobotNav/RobotNav/ESP32-DEVKITC-32U/ESP32-DEVKITC-32U.pdif
                                                    RobotNav/RobotNav/ESP32-DEVKITC-32U/ESP32-DEVKITC-32U.step
                                                    RobotNav/RobotNav/ESP32-DEVKITC-32U/how-to-import.htm    
                                          
```

### Licence MIT

Ces ressources et documents sont fournis "*En l'état*", sans aucun engagement de la part ni de l'ICAM 
ni de l'ARBL quant à leur validité / maintenance / correction / délivrance.

Inutile non plus de nous demander de vous fournir un circuit imprimé sorti de fonderie ou a fortiori 
une carte de navigation tout équipée.

C'est dans l'esprit du strict *do it yourself* que nous vous fournissons ces ressources.
Pour vos propres bouées il vous faudra retrousser vos manches, activer vos neurones (et dépenser vos propres deniers ;>))    
 