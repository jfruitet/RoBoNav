#IMPORT
import tkinter as tk
import socket
import time
import json
from datetime import datetime
import re

#INIT
screen = tk.Tk()

#SET SCREEN
screen.title("RoBoNav - Expérimentation")
#size
#background

#Frame - Screen
frame_screen = tk.LabelFrame(screen, borderwidth=4, relief="groove")
frame_screen.pack(fill="both", expand="yes")

#INPUT - Frame 1
frame_input = tk.LabelFrame(frame_screen, text="INPUT", borderwidth=2, relief="groove")
frame_input.grid(row=0, column=0, sticky="w")

#OUTPUT - Frame 2
frame_output = tk.LabelFrame(frame_screen, text="OUTPUT", borderwidth=2, relief="groove")
frame_output.grid(row=1, column=0, sticky="w")

#START - END - Frame 3
frame_commande = tk.LabelFrame(frame_screen, text="COMMANDE", borderwidth=2, relief="groove")
frame_commande.grid(row=0, column=1, sticky="w")

#AFFICHAGE 1 -Frame 4
frame_affichage_1 = tk.LabelFrame(frame_screen, text="MESSAGE BOUEE", borderwidth=2, relief="groove")
frame_affichage_1.grid(row=0, column=2, sticky="w")

zone_text_1 = tk.Text(frame_affichage_1, wrap=tk.WORD, width=50, height=15)
zone_text_1.grid(row=1, column=0, sticky="w")

#AFFICHAGE 2
frame_affichage_2 = tk.LabelFrame(frame_screen, text="CONSOLE", borderwidth=2, relief="groove")
frame_affichage_2.grid(row=1, column=2, sticky="w")

zone_text_2 = tk.Text(frame_affichage_2, wrap=tk.WORD, width=50, height=15)
zone_text_2.grid(row=1, column=0, sticky="w")

ESP32_IP = "172.20.10.2"
PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', PORT))  # écoute sur toutes les interfaces
sock.setblocking(False)

lecture_available = tk.BooleanVar(value=False)
send_available = tk.BooleanVar(value=False)
start_available = tk.BooleanVar(value=False)
input_available = tk.BooleanVar(value=True)

JSON = {"Lieu":"","Date":"","Test":
    {"ID":"",
     "commande_moteur":0,
     "vitesse_vent":0.0,
     "direction_vent":0,
     "distance":00,
     "mode":"consommation",
     "duree":000,
     "Iterations":[]}}
    
    
    
def init_display() :
    # INPUT
    # Label - Input
    tk.Label(frame_input, text="Date :").grid(row=0, column=0, sticky="w")
    tk.Label(frame_input, text="Lieu :").grid(row=1, column=0, sticky="w")
    tk.Label(frame_input, text="N°test :").grid(row=2, column=0, sticky="w")
    tk.Label(frame_input, text="").grid(row=3, column=0, sticky="w")
    tk.Label(frame_input, text="VENT :").grid(row=4, column=0, sticky="w")
    tk.Label(frame_input, text="Vitesse :").grid(row=5, column=0, sticky="w")
    tk.Label(frame_input, text="Direction :").grid(row=6, column=0, sticky="w")
    tk.Label(frame_input, text="").grid(row=7, column=0, sticky="w")
    tk.Label(frame_input, text="Distance à parcourir :").grid(row=8, column=0, sticky="w")
    tk.Label(frame_input, text="Commande moyeur :").grid(row=9, column=0, sticky="w")
    tk.Label(frame_input, text="").grid(row=10, column=0, sticky="w")
    tk.Label(frame_input, text="MODE :").grid(row=11, column=0, sticky="w")
    tk.Label(frame_input, text="").grid(row=13, column=0, sticky="w")
    
    tk.Label(frame_input, text="m/s").grid(row=5, column=3, sticky="w")
    tk.Label(frame_input, text="°").grid(row=6, column=3, sticky="w")
    tk.Label(frame_input, text="m").grid(row=8, column=3, sticky="w")
    tk.Label(frame_input, text="%").grid(row=9, column=3, sticky="w")

    # Input
    date = tk.Entry(frame_input, width=20)
    lieu = tk.Entry(frame_input, width=20)
    num_test = tk.Entry(frame_input,  width=20)
    vitesse_vent = tk.Entry(frame_input, width=20)
    direction_vent = tk.Entry(frame_input,  width=20)
    distance_a_parcourir = tk.Entry(frame_input,  width=20)
    commande_moteur = tk.Entry(frame_input,  width=20)

    date.grid(row=0, column=1, sticky="w")
    lieu.grid(row=1, column=1, sticky="w")
    num_test.grid(row=2, column=1, sticky="w")
    vitesse_vent.grid(row=5, column=1, sticky="w")
    direction_vent.grid(row=6, column=1, sticky="w")
    distance_a_parcourir.grid(row=8, column=1, sticky="w")
    commande_moteur.grid(row=9, column=1, sticky="w")
    
    # Choix du mode
    mode = tk.StringVar(value="Consommation")  # valeur par défaut
    
    tk.Radiobutton(frame_input, text="Consommation", variable=mode, value="Consommation").grid(row=11, column=1, sticky="w")
    tk.Radiobutton(frame_input, text="Dérive", variable=mode, value="Dérive").grid(row=12, column=1, sticky="w")
    
    # Bouton - Send
    tk.Button(frame_input, text="Valider", command=lambda: valider(date.get(),
                                                                lieu.get(),
                                                                num_test.get(),
                                                                vitesse_vent.get(),
                                                                direction_vent.get(),
                                                                distance_a_parcourir.get(),
                                                                commande_moteur.get(),
                                                                mode.get())).grid(row=14, column=1, sticky="w")
    
    # OUTPUT
    # Label - Output
    tk.Label(frame_output, text="Temps : ").grid(row=0, column=0, sticky="w")
    tk.Label(frame_output, text="U : ").grid(row=1, column=0, sticky="w")
    tk.Label(frame_output, text="I : ").grid(row=1, column=3, sticky="w")
    tk.Label(frame_output, text="E : ").grid(row=2, column=0, sticky="w")
    tk.Label(frame_output, text="P : ").grid(row=2, column=3, sticky="w")
    tk.Label(frame_output, text="").grid(row=3, column=0, sticky="w")
    tk.Label(frame_output, text="Position GPS : ").grid(row=4, column=0, sticky="w")
    tk.Label(frame_output, text="Lat : ").grid(row=5, column=0, sticky="w")
    tk.Label(frame_output, text="Lon : ").grid(row=5, column=2, sticky="w")
    tk.Label(frame_output, text="Cap : ").grid(row=6, column=0, sticky="w")
    tk.Label(frame_output, text="").grid(row=7, column=3, sticky="w")
    tk.Label(frame_output, text="Temps total : ").grid(row=8, column=0, sticky="w")
    tk.Label(frame_output, text="CMg : ").grid(row=9, column=0, sticky="w")
    tk.Label(frame_output, text="CMd : ").grid(row=9, column=2, sticky="w")
    
    tk.Label(frame_output, text=" ms").grid(row=0, column=2, sticky="w")  
    tk.Label(frame_output, text=" V").grid(row=1, column=2, sticky="w")
    tk.Label(frame_output, text=" A").grid(row=1, column=5, sticky="w")
    tk.Label(frame_output, text=" J").grid(row=2, column=2, sticky="w")
    tk.Label(frame_output, text=" W").grid(row=2, column=5, sticky="w")
    tk.Label(frame_output, text=" ms").grid(row=0, column=2, sticky="w")
    tk.Label(frame_output, text=" °").grid(row=6, column=2, sticky="w")
    
    # Output - Defaut
    tk.Label(frame_output, text=" 000").grid(row=0, column=1, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=1, column=1, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=1, column=4, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=2, column=1, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=2, column=4, sticky="w")
    tk.Label(frame_output, text=" 00.00000000").grid(row=5, column=1, sticky="w")
    tk.Label(frame_output, text=" 00.00000000").grid(row=5, column=3, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=6, column=1, sticky="w")
    tk.Label(frame_output, text=" 0000").grid(row=9, column=1, sticky="w")
    tk.Label(frame_output, text=" 0000").grid(row=9, column=3, sticky="w")
    
    # Boutons - Start & End
    tk.Button(frame_commande, text="START", command=start).grid(row=0, column=0, sticky="w")
    tk.Button(frame_commande, text="END", command=end).grid(row=0, column=1, sticky="w")
    
    tk.Frame(frame_commande, width=10, height=10, bg='red').grid(row=0, column=2, sticky="w")
    
    # Scrollbar - affichage 1
    scrollbar_1 = tk.Scrollbar(frame_affichage_1, command=zone_text_1.yview)
    scrollbar_1.grid(row=1,column=1, sticky="ns")
    zone_text_1.config(yscrollcommand=scrollbar_1.set)
    
    # Scrollbar - affichage 2
    scrollbar_2 = tk.Scrollbar(frame_affichage_2, command=zone_text_2.yview)
    scrollbar_2.grid(row=1,column=1, sticky="ns")
    zone_text_2.config(yscrollcommand=scrollbar_2.set)

def update_display(data):
    U = 0
    I = 0
    CMg = 0
    CMd = 0
    lat = 0
    lon = 0
    
    global time_start
    current_time =  time.time() - time_start
    
    match = re.search(r"U\s*:\s*([-\d.]+)\s*,\s*I\s*:\s*([-\d.]+)\s*,\s*CMg\s*:\s*([-\d.]+)\s*,\s*CMd\s*:\s*([-\d.]+)\s*,\s*Lat\s*:\s*([-\d.]+)\s*,\s*Lon\s*:\s*([-\d.]+)", data)
    if match:
        U = float(match.group(1))
        I = float(match.group(2))
        CMg = float(match.group(3))
        CMd = float(match.group(4))
        lat = float(match.group(5))
        lon = float(match.group(6))
        
        tk.Label(frame_output, text=round(current_time,3)).grid(row=0, column=1, sticky="w")
        tk.Label(frame_output, text=lat).grid(row=5, column=1, sticky="w")
        tk.Label(frame_output, text=lon).grid(row=5, column=3, sticky="w")
        tk.Label(frame_output, text=U).grid(row=1, column=1, sticky="w")
        tk.Label(frame_output, text=I).grid(row=1, column=4, sticky="w")
        tk.Label(frame_output, text=CMg).grid(row=9, column=1, sticky="w")
        tk.Label(frame_output, text=CMg).grid(row=9, column=3, sticky="w")
    else:
        print("Coordonnées non trouvées.")
    
    if U != 0 :  
        JSON["Test"]["Iterations"].append(update_JSON(U,I,CMg,CMd,lat,lon,round(current_time,3)))
    

def update_JSON(U,I,CMg,CMd,lat,lon,time) :
    iteration = {"#":0, "time":0, "COG":100, "U":12.5, "I":2, "CMg":50, "CMd":50,"Lat":47.12345678, "Lon":-1.12345678}
    print(len(JSON["Test"]["Iterations"]))
    
    iteration['time'] = time
    iteration["#"] = len(JSON["Test"]["Iterations"])
    iteration["U"] = U
    iteration["I"] = I
    iteration["CMg"] = CMg
    iteration["CMd"] = CMd
    iteration["Lat"] = lat
    iteration["Lon"] = lon
    
    ajout_msg_console(iteration, 2)

    return iteration
    
    
    

def valider(date,lieu,id_test,vvent,dvent,d,cm,mode):
    # print(date,lieu,vvent,dvent,d,cm)
    etat_date, etat_lieu, etat_id_test, etat_vvent, etat_dvent, etat_d, etat_cm = False, False, False, False, False, False, False
    
    if input_available.get() :
        # vérification des informations
        # DATE
        if date != "" :
            try:
                datetime.strptime(date, "%d/%m/%Y")
                etat_date = True
                tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=0, column=4, sticky="w")
                JSON["Date"] = date
            except ValueError : 
                etat_date = False
                tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=0, column=4, sticky="w")
        else :  
            etat_date = False
            tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=0, column=4, sticky="w")
        
        # LIEU
        if lieu != "" :
            tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=1, column=4, sticky="w")
            etat_lieu = True
            JSON["Lieu"] = lieu
        else : 
            etat_lieu = False
            tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=1, column=4, sticky="w")
                
        # ID_test
        if id_test != "" :
            try : 
                id_test = int(id_test)
                tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=2, column=4, sticky="w")
                etat_id_test = True
                JSON["Test"]["ID"] = id_test
            except ValueError :
                tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=2, column=4, sticky="w")
                etat_id_test = False
        else :
            tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=2, column=4, sticky="w")
            etat_id_test = False
        
        #VVENT
        if vvent != "" :
            try : 
                vvent = int(vvent)
                tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=5, column=4, sticky="w")
                etat_vvent = True
                JSON["Test"]["vitesse_vent"] = vvent
            except ValueError :
                tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=5, column=4, sticky="w")
                etat_vvent = False
        else : 
            tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=5, column=4, sticky="w") 
            etat_vvent = False    
        
        #DVENT
        if dvent != "" :
            try : 
                dvent = int(dvent)
                tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=6, column=4, sticky="w")
                etat_dvent = True
                JSON["Test"]["direction_vent"] = dvent
            except ValueError :
                tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=6, column=4, sticky="w")
                etat_dvent = False
        else : 
            tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=6, column=4, sticky="w")
            etat_dvent = False
        
        #DISTANCE
        if d != "" :
            try : 
                d = int(d)
                tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=8, column=4, sticky="w")
                etat_d = True
                JSON["Test"]["distance"] = d
            except ValueError :
                tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=8, column=4, sticky="w")
                etat_d = False
        else : 
            tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=8, column=4, sticky="w")
            etat_d = False
        
        #COMMANDE MOTEUR
        if mode == "Consommation" :
            if cm != "" :
                try : 
                    cm = int(cm)
                    if cm <= 100 and cm >= 0 :
                        tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=9, column=4, sticky="w")
                        etat_cm = True
                        JSON["Test"]["commande_moteur"] = cm
                except ValueError :
                    tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=9, column=4, sticky="w")
                    etat_cm = False
            else : 
                etat_cm = False
                tk.Frame(frame_input, width=10, height=10, bg='red').grid(row=9, column=4, sticky="w")
        else : 
            tk.Frame(frame_input, width=10, height=10, bg='green').grid(row=9, column=4, sticky="w")
            etat_cm = True
        
        JSON["Test"]["mode"] = mode
        
        if etat_date and etat_lieu and etat_id_test and etat_vvent and etat_dvent and etat_d and etat_cm :
            # ajout des valeurs initiale au JSON
            send_available.set(True) 
            ajout_msg_console(JSON,2)           
        else : 
            send_available.set(False)
            ajout_msg_console("Au moins un INPUT est invalide",2)
            
        # IF valeurs valide envoye de la commande moteur si en mode consommation    
        if send_available.get() :
            envoie(str(mode))
            #while 
            if mode == "Consommation" :
                ajout_msg_console("Commande moteur envoyée",2)
                envoie(str(cm))
            start_available.set(True)
            ajout_msg_console("START available",2)
        send_available.set(False)

def ajout_msg_console(msg, num_console) :
    msg = str(msg)
    if num_console == 1 :
        zone_text_1.insert(tk.END, msg + "\n")  # Ajoute à la fin
        zone_text_1.see(tk.END)
    elif num_console == 2 :
        zone_text_2.insert(tk.END, msg + "\n")  # Ajoute à la fin
        zone_text_2.see(tk.END)
    
def lecture_step():
    
    data = ""
    try:
        data, addr = sock.recvfrom(8192)
        data = data.decode()
        # data = "11111 [EXPERIENCE] Lat : 47.12345678, Lon : -1.12345678"
        ajout_msg_console(f"{data}",1)
        #return data
        
    except BlockingIOError:
        pass  # Pas de données pour le moment
    if lecture_available.get() :
        update_display(data)

    screen.after(500, lambda: lecture_step())
     
def start():
    if start_available.get() :
        global time_start 
        time_start = time.time()
        input_available.set(False)
        ajout_msg_console("En cours d'enregistrement des données",2)
        # Lancement du chrono
        lecture_available.set(True)
        tk.Frame(frame_commande, width=10, height=10, bg='green').grid(row=0, column=2, sticky="w")
    else : ajout_msg_console("Appuyer sur VALIDER avant de commencer",2)

def end():
    #-----------------------------------------------------------------------------
    global time_start
    duree = round(time.time()-time_start)
    JSON["Test"]["duree"]=duree
    time_start = 0
    #-----------------------------------------------------------------------------
    ajout_msg_console("Fin d'enregistrement des données",2)
    tk.Frame(frame_commande, width=10, height=10, bg='red').grid(row=0, column=2, sticky="w")
    #-----------------------------------------------------------------------------
    lecture_available.set(False)
    start_available.set(False)
    send_available.set(False)
    input_available.set(True)
    #-----------------------------------------------------------------------------
    tk.Label(frame_output, text=" 00").grid(row=1, column=1, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=1, column=4, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=2, column=1, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=2, column=4, sticky="w")
    tk.Label(frame_output, text=" 00.00000000").grid(row=5, column=1, sticky="w")
    tk.Label(frame_output, text=" 00.00000000").grid(row=5, column=3, sticky="w")
    tk.Label(frame_output, text=" 00").grid(row=6, column=1, sticky="w")
    tk.Label(frame_output, text=" 0000").grid(row=9, column=1, sticky="w")
    tk.Label(frame_output, text=" 0000").grid(row=9, column=3, sticky="w")
    #-----------------------------------------------------------------------------
    now = datetime.now()
    filename = now.strftime("test_ROBONAV_%Y-%m-%d_%H-%M-%S.json")
    
    # Enregistrer le fichier JSON avec indentations jolies
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(JSON, f, indent=4)
    
    ajout_msg_console(f"Fichier JSON enregistré sous : {filename}", 2)

def envoie(msg):
    ajout_msg_console("message envoyé : "+ msg,2)
    sock.sendto(msg.encode(), (ESP32_IP, PORT))

if __name__ == '__main__' :
    global time_start
    time_start = 0
    init_display()
    lecture_step()
    screen.mainloop()