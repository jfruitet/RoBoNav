// Algorithme de Jean Fruitet pour le FailSafe de la bouée.

// Tableau d’entiers 9 x 9
automate_fly_safe = [ 0, 0, 0, 0, 0, 0, 0, 0, 0,   // STOP
                      0, 0, 0, 0, 0, 0, 0, 2, 2,   // INITIAL
                      0, 0, 0, 0, 0, 0, 0, 3, 4,   // HOME
                      0, 0, 7, 3, 0, 6, 7, 3, 4,   // MANUEL
                      0, 0, 7, 3, 4, 3, 7, 3, 4,   // AUTONOME
                      0, 8, 7, 3, 5, 3, 7, 3, 5,   // GOTO
                      0, 8, 7, 3, 6, 3, 7, 6, 6,   // STAY
                      0, 8, 7, 3, 7, 3, 7, 7, 7,   // RTH
                      0, 8, 8, 8, 8, 8, 8, 3, 4 ]; // IDLE

int etat_courant = 0;
int etat_suivant = 0;

// Etats (numéro de ligne)
int index_etat_stop = 0;
int index_etat_initial = 1;
int index_etat_home = 2;
int index_etat_manuel = 3;
int index_etat_autonome = 4;
int index_etat_goto = 5;
int index_etat_stay = 6;
int index_etat_rth = 7;
int index_etat_idle = 8;

// Transitions (numéro de colonne)
int index_error_alim = 0;
int index_error_tens1 = 1;
int index_error_tens2 = 2;
int index_error_gps = 3;
int index_error_reception = 4;
int index_error_derive = 5;
int index_error_obstacle = 6;
int index_mode_manuel = 7;
int index_mode_autonome = 8;
boolean error_alim, error_tens1, error_tens2, error_gps, error_reception, error_obstacle, error_derive;

// Alimentation
float tension_courante = 14.5; // Batterie LiPo 2S
float tens_min0 = 9.6;
float tens_min1 = 10.5;
float tens_min2 = 12;

// GPS
#define MAXERREUR_GPS = 1000.0; // Bon là j’improvise…
boolean ok_gps = true;
float taux_erreur_gps = 0.0;
GPS_Class target_gps; // Position GPS à atteindre
// sur le modèle target_gps.lon, target_gps.lat

// Réception
#define MAXERREUR_RECEPTION = 1000.0; // Bon là j’improvise…
boolean ok_receptions = true;
float taux_erreur_reception = 0.0;
// Dérive
boolean ok_derive = false;

// Obstacle
boolean ok_obstacle = fale;

// Numéro de bouée
numero_bouee = 1;
// Radiocommande ou smartphone ?
ok_radiocommande = false;
ok_wifi = false;
// Mode ?
#define VALEUR_ON = 1500; // Durée min du signal PWM état ON
#define VALEUR_OFF = 800; // Durée min du signal PWM état OFF

#define MODE_MANUEL = true;    // Pour forcer le changement de mode
#define MODE_AUTONOME = false; // Voir la fonction set_mode(boolean);
boolean mode_courant = true;   // Mode manuel par défaut
valeur_canalmode = 4;          // Canal radio qui active le mode manuel / autonome sur les radiocommandes
// Pour la liaison WiFi on interprète un message de type “NUM_BOUEE,MODE,1;” ou une chaîne JSON

// Temporisations
int compteur_gps = 0;
int compteur_reception = 0;
int compteur_derive = 0;
int compteur_obstacle = 0;
int tempo_gps = 5000;       // Micro secondes
int tempo_reception = 5000; // Microsecondes
int tempo_derive = 5000;    // Micro secondes
int tempo_obstacle = 5000;  // Micro secondes
// Alertes
string msg_erreur = "";
#define ALERTE_GPS = "ERREUR GPS";
#define ALERTE_RECEPTION = "ERREUR RECEPTION";
#define ALERTE_DERIVE = "ERREUR DERIVE";
#define ALERTE_OBSTACLE = "ERREUR OBSTACLE";
#define ALERTE_TENSION1 = "ERREUR TENSION TRES FAIBLE";
#define ALERTE_TENSION2 = "ERREUR TENSION FAIBLE";
#define ALERTE_ALIMENTATION = "ERREUR ALIMENTATION GENERALE";

// Retourne l’état suivant en fonction de l’état courant et de la transition
// Formule d’adressage indirect : num ligne * (nb_colonne+1) + num_colonne
// —----------------------
function get_etat_suivant(int etat_courant, int transition){
    return (automate_fly_safe[etat_courant * 10 + transition])}
// Initialise toutes les variables
// Initialise le WiFi et le GPS
// Lit les valeurs fournies par la carte RoBoNav
// —---------------------
function initialise()
{
    etat_courant = 0;

    // Transitions
    error_alim = false;
    error_tens1 = false;
    error_tens2 = false;
    error_gps = false;
    error_reception = false;
    error_obstacle = false;
    error_derive = false;

    // Pseudo transitions
    mode_autonome = false;
    tension_courante = get_tension(); // Output Circuit Intégré RoBoNav
                                      // Dérive
    compteur_derive = 0;
    // Obstacle
    compteur_obstacle = 0;

    ok_gps = init_gps(); // Output Circuit Intégré RoBoNav
    if (ok_gps)
    {
        set_home_gps(); // Enregistre la position de départ “HOME”
    }
    else
    {
        println(“Erreur GPS.STAY et RTH indisponibles.Bouée arrêtée…”);
        commande_EteindreBouee();
        die(); // Arrêt du programme
    }
    ok_wifi = init_wifi();                   // Output Circuit Intégré RoBoNav
    ok_radiocommande = init_radiocommande(); // Output Circuit Intégré RoBoNav

    // WiFi et smartphone prioritaire sur la radio commande
    // Au démarrage on ne souhaite pas de mise à l’eau
    // si ni la connexion WiFi ni la liaison radio ne fonctionnent…
    if (ok_wifi)
    {
        println(“WiFi activé.Utilisez le smartphone pour piloter la bouée N° % i”, numero_bouee);
    }
    else if (ok_radiocommande)
    {
        println(“Radiocommande connectée.Utilisez la pour piloter la bouée N° % i”, numero_bouee);
    }
    else
    {
        println(“Aucune connexion radio.Bouée arrêtée… ”);
        commande_EteindreBouee();
        die();
    }

    // Mode manuel par défaut
    mode_manuel = get_mode();
    // Traite une bascule
    // envoyée par smartphone (WiFi) ou par la radio commande
}
// —----------------------
// retourne les erreurs
function get_errors()
{
    message_erreur =””;
    on_error = false;

    // Tension
    error_alim = false;
    error_tens1 = false;
    error_tens2 = false;
    if (tension_courante < tens_min0)
    {
        error_alim = true;
        on_error = true;
        msg_erreur = ALERTE_ALIMENTATION;
    }
    else if (tens_min0 <= tension_courante < tens_min1)
    {
        error_tens1 = true;
        on_error = true;
        msg_erreur = ALERTE_TENSION1;
    }
    else if (tension_courante < tens_min2)
    {
        error_tens2 = true;
        on_error = true;
        msg_erreur = ALERTE_TENSION2;
    }

    // GPS
    error_gps = false;
    if (!ok_gps || (taux_erreur_gps > MAXERREUR_GPS))
    {
        error_gps = true;
        msg_erreur = ALERTE_GPS;
    }
    // Réception
    error_reception = false;
    if ((!ok_wifi && !ok_radiocommande) || (taux_erreur_reception > MAXERREUR_RECEPTION))
    {
        error_reception = true;
        on_error = true;
        msg_erreur = ALERTE_RECEPTION;
    }
    // Dérive
    erreur_derive = false;
    if (ok_derive)
    {
        if (compteur_derive > MAXERREUR_DERIVE)
        {
            error_derive = true;
            on_error = true;
            msg_erreur = ALERTE_DERIVE;
            compteur_derive = 0;
        }
        else
        {
            compteur_derive++;
        }
    }
    // Obstacle
    error_obstacle = false;
    if (ok_obstacle)
    {
        if (compteur_obstacle > MAXERREUR_DERIVE))
            {
                error_obstacle = true;
                on_error = true;
                msg_erreur = ALERTE_OBSTACLE;
                compteur_obstacle = 0;
            }
        else
        {
            compteur_obstacle++;
        }
    }
    return on_error;
}

/*****************************************************

        Début du programme

******************************************************/

initialise();
void main()
{
    while (true)
    { // Boucle infinie
        // Outputs CI RoBoNav
        tension_courante = get_tension();   // Tension électrique batterie
        taux_erreur_gps = get_gps();        // Délivre le taux d’erreur GPS
        taux_erreur_reception = get_wifi(); // Délivre erreurs WiFi
        ok_derive = get_derive();           // déplacement non nul en mode STAY
        ok_obstacle = get_obstacle();       // déplacement nul en mode GOTO/RHT

        // Positionne les erreurs
    if (get_errors(){ // Erreur détectée
            println(message_erreur);
            // Fournit l’état suivant
            if (error_alim == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_alim);
            }
            else if (error_tens1 == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_tens1);
            }
            else if (error_tens2 == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_tens2);
            }
            else if (error_gps == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_gps);
            }
            else if (error_reception == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_reception);
            }
            else if (error_derive == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_derive);
            }
            else if (error_obstacle == true)
            {
                etat_suivant = get_etat_suivant(etat_courant, index_error_obstacle);
            }
            else if (mode_manuel == false)
            { // Autonome
                etat_suivant = get_etat_suivant(etat_courant, index_mode_autonome);
            }
            else
            {
                etat_suivant = get_etat_suivant(etat_courant, index_mode_manuel);
            }
	}
switch (etat_suivant){
        case index_etat_stop:
            println("STOP");
            eteindre_bouee();
            break;

        case index_etat_initial:
            println("INITAL");
            reinitialisation_bouee();
            break;

        case index_etat_home:
            println("HOME");
            set_home_gps();
            break;
        case index_etat_manuel:
            println("MANUEL");
            set_mode(MODE_MANUEL);
            dirige_bouee(); // Commandes de déplacement ?
            break;
        case index_etat_autonome:
            println("AUTONOME");
            set_mode(MODE_AUTONOME);
            deplacement_autonome(); // Commande ?
            break;
        case index_etat_goto:
            println("GOTO POSITION GPS");
            goto_target();
            break;
        case index_etat_stay:
            println("STAY POSITION GPS");
            if (etat_courant == etat_suivant)
            {
                stay_position();
            }
            else
            {
                init_stay_position();
            }
            break;
        case index_etat_rth:
            println("RTH POSITION GPS HOME");
            if (etat_courant == etat_suivant)
            {
                rth_home();
            }
            else
            {
                init_rth_position();
                rth_home();
            }

            break;
            default println("IDLE");
            mise_en_veille_profonde();
            break;
} // switch
// On avance
	etat_courant = etat_suivant;
    } // Fin gestion de l’erreur
    // Gestion des commandes
    // Modifie le comportement du programme en fonction des interruptions
    // Activées sur le smartphone ou sur la radiocommande
    get_commandes();

    // Temporisation
    delay(500); // 0,5 seconde d’attente
} // while
} // main
/***************************************************

Fonctions de gestion de la bouée

****************************************************/

// —----------------------
// Eteindre la bouée
function eteindre_bouee()
{
    //  A Programmer
    die();
}
// —----------------------
// Reset de toutes les variables d’état.
function reinitialisation_bouee()
{
    //  A Programmer
}
// —----------------------
// Enregistrement de la position GPS du ponton de départ.
function set_home_gps()
{
    // Enregistre dans la mémoire de la carte de navigation la position GPS.
    home_pos_gps = get_GPS();
    save_card(“HOME”, home_pos_gps); // ça pourrait être un cookie.
}
// —----------------------
// Déplacement autonome vers la position GPS assignée.
// La variable globale GPS_Class target_gps doit être attribuée
// La classe GPS doit fournir un record Lon, Lat en degrés géographiques décimaux.

function goto_target()
{
    global target_gps;
    set_mode(MODE_AUTONOME);
    // Activer les interruptions ad hoc sur la carte de navigation
    // Pour orienter la bouée vers sa destination GPS et lancer les moteurs
    //
    pos_gps = get_GPS();
    // Implanter un PID ici
    double d_but = diff_pos_gps(pos_gps, target_gps);
    if (d_but > 1000)
    { // 10 mètres
        avancer(PLEINE_PUISSANCE);
    }
    else
        (d_but > 200)
        { // 2 mètres
            avancer(MOYENNE_PUISSANCE);
        }
    else(d_but > 100)
    { // 1 mètre
        avancer(FAIBLE_PUISSANCE);
    }
    else
    {
        init_stay_position();
    }
}
// —----------------------
// Enregistre à la position GPS courante.
function init_stay_position()
{
    global target_gps;
    set_mode(MODE_AUTONOME);
    target_gps = get_GPS();
    save_card(“STAY”, target_gps); // ça pourrait être un cookie.
    stay_position();
}
// —----------------------
// Maintien autonome à la position assignée.
// Appel seulement après initialisation de la variable gloabale target_gps
function stay_position()
{
    global target_gps
        set_mode(MODE_AUTONOME);
    pos_gps = get_GPS();
    if (diff_pos_gps(pos_gps, target_gps) > DELTA_POS_GPS)
    {
        goto_target();
    }
}
// —----------------------
// Mesure de distance entre deux positions GPS
// On n’a vraiment pas besoin d’utiliser le calcul du Grand Cercle
// vu les dimensions du plan d’eau
function diff_pos_gps(pos_gps1, pos_gps2)
{
    double poslon1 = conversion_numerique(pos_gps1.lon);
    double poslat1 = conversion_numerique(pos_gps1.lat);
    double poslon2 = conversion_numerique(pos_gps2.lon);
    double poslat2 = conversion_numerique(pos_gps2.lat);
    double d = SQRT((poslon1 - poslon2) * (poslon1 - poslon2) + (poslat1 - poslat2) * (poslat1 - poslat2));
    return d;
}
// —----------------------
// Enregistre la position de départ
function set_home_gps()
{
    home_gps = get_GPS();
    save_card(“HOME”, home_gps); // ça pourrait être un cookie.
}

// —----------------------
// Lit la position de départ
function get_home_gps()
{
    return read_card(“HOME”, home_gps); // ça pourrait être un cookie.
}
// —----------------------
function init_rth_position()
{
    global target_gps;
    set_mode(MODE_AUTONOME);
    target_gps = get_home_gps();
    goto_target();
}

// —----------------------
// Retour autonome à la position HOME.
// Après initialisation de la variable globale target_gps
// par init_rth_position()
function rth_home()
{
    set_mode(MODE_AUTONOME);
    goto_target();
}
// —----------------------
// Moteurs coupés. Mise en veille en mode économie d’énergie.
function mise_en_veille_profonde()
{
    // A Programmer
}
// —----------------------
// Interprète les entrées sur le récepteur ou sur le WiFi et dirige la bouée.
function dirige_bouee()
{
    // A Programmer
}
// —----------------------
// Déplace la bouée dans la direction ad_hoc la bouée.
function avancer(int puissance_moteurs)
{
    global target_gps;
    pos_gps = get_GPS();
    float direction = calcule_angle_nord(pos_gps, target_gps);
    double distance = diff_pos_gps(pos_gps, target_gps)
    {
        commande_RotationBouee(direction);
        aller_tout_droit(puissance_moteurs, distance);
    }

    // —----------------------
    // Moduler la puissance en fonction de la distance à parcourir
    function aller_tout_droit(puissance_moteurs, float distance)
    {
        float puissance = 0.0;
        if (distance < 500)
        {
            // 50 cm, on sort de la précision GPS
            // Utiliser l’accéléromètre ?
        }
        else
            (distance < 1000)
            {
                // 1 mètre
                puissance = (float)puissance_moteurs / 2.0;
            }
        else
        {
            // 1 mètre
            puissance = (float)puissance_moteurs;
        }
        commande_AvanceBouee(puissance);
    }
    // —----------------------
    // Boucle sur l’état actif courant (STAY, RTH, GOTOPOS).
    function deplacement_autonome()
    {
        // A Programmer
    }
    /* *****************************************

        Commandes à programmer
        dans l’environnement de la carte
        contrôleur

    * ***************************************** */

    // Output Circuit Intégré RoBoNav
    // Fournit la tension de la batterie pour les moteurs
    // —----------------------
    function get_tension(){
    … }
    // Initialise le positionnement GPS
    // Met à jour la variable GPS Home
    // —----------------------
    function init_gps(){
    … }

    // Met à jour la variable GPS Home
    // —----------------------
    function get_GPS()
    {
        return (...); // interruption en lecture sur l’entrée connectée au chip GPS
    }
    // Initialise la communication WiFi
    // —----------------------
    function init_wifi(){
    … }
    // Initialise la communication par radiocommande
    // —----------------------
    function init_radiocommande(){
	… }
    // Récupère le mode par défaut (lecture sur un interrupteur de la radiocommande ou du smartphone
    // Commande adressée par radiocommande ou par WiFi
    // —----------------------
    function get_mode()
    {
        // positionne le mode_courant à true ou à false
        // TRUE : mode manuel
        if (ok_radiocommande)
        {
            // Lire le canal 4
            if (valeur_canalmode > VALEUR_ON)
            {
                mode_courant = MODE_MANUEL;
            }
            else
            {
                mode_courant = MODE_AUTONOME;
            }
        }
        else if (ok_wifi)
        {
            // Interpréter la commande reçue
            if (wifi_message == “NUM_BOUEE, MODE, 1;”)
            {
                mode_courant = MODE_MANUEL;
            }
            else
            {
                mode_courant = MODE_AUTONOME;
            }
        }
        …
    }

    // —----------------------
    function set_mode(boolean mode)
    {
        // positionne le mode_courant à true ou à false
        mode_courant = mode;
    }
    // —----------------------
    // Lecture des ordres envoyés par WiFi ou par la radiocommande
    function get_commandes()
    {
        // A Programmer
    }
    // —----------------------
    function commande_EteindreBouee()
    {
        // A programmer
    }

    // —---------------------
    function commande_AvanceBouee(float puissance)
    {
        // A programmer
    }

    // —---------------------
    function commande_RotationBouee(float direction)
    {
        // A programmer
    }