package K
  constant Real g = 9.81 "Accelerazione di gravita [m/s^2]";
  constant Real T = 0.02 "Passo di integrazione [s]";
  constant Integer N = 10 "Numero totale droni nello swarm";

  // Sensori LiDAR & Griglia 27 Bin
  constant Real FOV_horiz = 3.14159265359 "FOV Orizzontale 180 gradi [rad]";
  constant Real FOV_vert = 1.0471975512 "FOV Verticale 60 gradi [rad]";
  constant Real sensorRange = 20.0 "Gittata massima LiDAR [m]"; 

  constant Integer grid_H = 9 "Bin orizzontali";
  constant Integer grid_V = 3 "Bin verticali";
  constant Integer grid_size = 27 "Dimensione totale griglia (9x3)";

  // Mappa e Confinamento
  constant Real dome_radius = 50.0 "Raggio cupola [m]";
  constant Real dome_center[3] = {0.0, 0.0, 0.0};
  constant Real dome_max_height = 50.0;

  // Fisica Drone
  constant Real m = 5.0 "Massa drone [kg]";
  constant Real Cd = 0.5 "Resistenza aria";
  constant Real Area_side = 0.15 "Superficie frontale [m^2]";
  constant Real Area_top = 0.30 "Superficie superiore [m^2]";
  constant Real rho = 1.225 "Densita aria [kg/m^3]";
  
  constant Real V_nom = 22.2 "Tensione nominale pacco 6S [V]";
  constant Real Bat_Capacity_Ah = 16.0 "Capacita totale batteria [Ah]";
  constant Real Bat_Energy_Wh = 355.2 "Energia nominale stoccata [Wh]";
  constant Real Bat_Energy_J = 1278720.0 "Energia totale stoccata [Joule]";
  constant Real P_hover = 750.0 "Potenza elettrica stimata in hovering statico [W]";
  
  /*
  P_AVIONICS = 15.0 [W] — Consumo Elettronica di Bordo
È la potenza elettrica costante assorbita dal sistema che non produce spinta: Flight Controller, Companion Computer (su cui gira l'AI), LiDAR, sensori di bordo e ponte radio/telemetria. È un carico parassita indipendente dallo stato di moto dei motori.


E_TOTAL_JOULES = 1278720.0 [J] — Capacità Nominale della Batteria
Rappresenta l'energia massima accumulabile nella batteria LiPo 6S da 16000 mAh, ricavata dalla relazione $E = V \cdot I \cdot t$


P_HOVER = 750.0 [W] — Potenza Elettrica di Hovering
È la potenza assorbita dall'impianto propulsivo (motori + ESC) per mantenere il drone da 5 kg in hovering stazionario


Una batteria 6S da 16000 mAh accumula circa $1.28 \text{ MJ}$ ($1\,278\,720 \text{ J}$) di energia.In un volo rettilineo senza ostacoli di 15-20 secondi:Potenza media assorbita: $\approx 750\text{--}800 \text{ W}$Energia consumata: $750 \text{ W} \times 20 \text{ s} = 15\,000 \text{ J}$Incidenza sulla batteria: $\frac{15\,000 \text{ J}}{1\,278\,720 \text{ J}} \times 100 \approx \mathbf{1.17\%}$


*/
  //E_TOTAL_JOULES = 1278720.0 [J] — Capacità Nominale della Batteria
//Rappresenta l'energia massima accumulabile nella batteria LiPo 6S da 16000 mAh, ricavata dalla relazione $E = V \cdot I \cdot t$:







  
  constant Real P_avionics = 15.0 "Assorbimento costante sensoristica e computer di bordo [W]";
  constant Real Max_Usable_SoC = 0.85 "Limite di scarica di sicurezza (85% dell'energia totale)";
  
end K;
