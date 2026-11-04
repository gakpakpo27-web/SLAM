# Exploration autonome – résultats en simulation

Salle simulée de 13,5 × 9,8 m en L, avec meuble central, armoire et 60 pieds de chaises de 4 cm. Le départ est tiré au hasard. Les vitesses réelles du robot diffèrent du réglage de 0,8 à 1,2 fois, et l’odométrie a ±10 % d’erreur plus une petite dérive. Le robot ne connaît pas la salle : il ne voit que ses scans. Commande : `node bench_exploration.js`.

| Graine | Départ (x ; y ; cap) | Vitesse réelle | Fin | Couverture | Chocs | Ordres | Distance | Positions | Erreur de pose médiane / max | Arrivée à |
|---|---|---|---|---|---|---|---|---|---|---|
| 20 | 12.5 ; 2.7 ; -121° | × 0.91 | cartographie terminée, robot revenu au départ | 100 % | 0 | 146 | 36 m | 43 | 2.7 cm / 3.5 cm | 11.7 cm du départ |
| 21 | 5.9 ; 4.5 ; -78° | × 0.95 | cartographie terminée, robot revenu au départ | 100 % | 0 | 149 | 35 m | 50 | 1.7 cm / 2.8 cm | 14.4 cm du départ |
| 22 | 6.1 ; 6.6 ; 88° | × 0.81 | cartographie terminée, robot revenu au départ | 100 % | 0 | 153 | 38 m | 52 | 1.4 cm / 3.2 cm | 8.9 cm du départ |
| 23 | 1.7 ; 1.1 ; 48° | × 0.92 | cartographie terminée, robot revenu au départ | 100 % | 0 | 135 | 35 m | 47 | 2.1 cm / 4.2 cm | 22.1 cm du départ |
| 24 | 4.7 ; 9.2 ; -74° | × 0.84 | cartographie terminée, robot revenu au départ | 99 % | 0 | 224 | 48 m | 40 | 2.5 cm / 3.7 cm | 16.1 cm du départ |
| 25 | 5.8 ; 4.6 ; -59° | × 0.90 | cartographie terminée, robot revenu au départ | 100 % | 0 | 179 | 41 m | 53 | 1.8 cm / 3.2 cm | 7.1 cm du départ |
| 26 | 6.9 ; 7.2 ; 4° | × 0.80 | cartographie terminée, robot revenu au départ | 100 % | 0 | 182 | 40 m | 57 | 2.0 cm / 3.2 cm | 9.9 cm du départ |
| 27 | 2.6 ; 8.0 ; -93° | × 0.91 | cartographie terminée, robot revenu au départ | 100 % | 0 | 148 | 34 m | 50 | 2.1 cm / 3.9 cm | 19.5 cm du départ |
| 28 | 5.9 ; 6.9 ; 85° | × 1.10 | cartographie terminée, robot revenu au départ | 100 % | 0 | 118 | 27 m | 30 | 1.9 cm / 3.1 cm | 21.5 cm du départ |
| 29 | 3.6 ; 4.0 ; -55° | × 0.94 | cartographie terminée, robot revenu au départ | 100 % | 0 | 168 | 36 m | 54 | 1.4 cm / 2.8 cm | 19.2 cm du départ |
| 30 | 2.8 ; 1.9 ; -24° | × 1.15 | cartographie terminée, robot revenu au départ | 99 % | 0 | 137 | 30 m | 47 | 2.1 cm / 3.3 cm | 6.4 cm du départ |
| 31 | 1.1 ; 4.3 ; -56° | × 0.83 | cartographie terminée, robot revenu au départ | 100 % | 0 | 175 | 38 m | 48 | 2.2 cm / 3.5 cm | 9.8 cm du départ |

**Bilan.** Cartographie terminée dans 12/12 cas, avec retour au départ dans 12/12 cas. Couverture médiane 100 % (minimum 99 %). Chocs : 0. Erreur de pose médiane 2.1 cm, maximum 4.2 cm. 153 ordres et 36 m parcourus en médiane. La couverture est la part de la surface libre réelle que la carte marque libre.

## Contrôle du montage du lidar (premiers mouvements)

| Cas | Ordres avant l’arrêt | Chocs | Message |
|---|---|---|---|
| lidar tourné de 90° | 5 | 0 | arrêt : le robot avance à -91° de l’avant du lidar : le 0° du lidar n’est pas l’avant du robot. Mets le décalage d’angle à 90° de plus dans les options d’affichage, puis relance. |
| lidar tourné de 180° | 5 | 0 | arrêt : le robot avance à -176° de l’avant du lidar : le 0° du lidar n’est pas l’avant du robot. Mets le décalage d’angle à 176° de plus dans les options d’affichage, puis relance. |
| scan en miroir | 5 | 0 | arrêt : les rotations mesurées sont à l’envers de la commande : le scan est en miroir. Coche (ou décoche) « miroir » dans les options d’affichage, puis relance. |

## Obstacles plus bas que le lidar : apport du capteur ultrason avant

Même salle, avec 50 pieds de chaises et 8 cartons de 30 × 25 cm plus bas que la nappe du lidar. Le lidar ne les voit pas ; l’ultrason sur servo les voit. Ultrason simulé : 13 directions de -90° à +90°, cône de ±10°, portée 3 m. Pare-chocs du firmware : arrêt si un obstacle est à moins de 15 cm dans le couloir du robot en marche avant.

| | Terminées | Chocs | Couverture moyenne |
|---|---|---|---|
| Lidar seul | 7/8 | 48 | 100 % |
| Lidar + ultrason | 8/8 | 0 | 100 % |

| Graine | Lidar seul : fin | chocs | Lidar + ultrason : fin | chocs |
|---|---|---|---|---|
| 40 | cartographie terminée, robot revenu au départ | 3 | cartographie terminée, robot revenu au départ | 0 |
| 41 | cartographie terminée, robot revenu au départ | 9 | cartographie terminée, robot revenu au départ | 0 |
| 42 | cartographie terminée, robot revenu au départ | 0 | cartographie terminée, robot revenu au départ | 0 |
| 43 | cartographie terminée, robot revenu au départ | 8 | cartographie terminée, robot revenu au départ | 0 |
| 44 | cartographie terminée, robot revenu au départ | 4 | cartographie terminée, robot revenu au départ | 0 |
| 45 | cartographie terminée, robot revenu au départ | 9 | cartographie terminée, robot revenu au départ | 0 |
| 46 | cartographie terminée, robot revenu au départ | 4 | cartographie terminée, robot revenu au départ | 0 |
| 47 | arrêt : le robot n’avance plus alors qu’il en reçoit l’ordre, malgré plusieurs contournements (coincé, roue qui patine ?) | 11 | cartographie terminée, robot revenu au départ | 0 |
