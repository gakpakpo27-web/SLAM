# Exploration autonome – résultats en simulation

Salle simulée de 13,5 × 9,8 m en L, avec meuble central, armoire et 60 pieds de chaises de 4 cm. Le départ est tiré au hasard. Les vitesses réelles du robot diffèrent du réglage de 0,8 à 1,2 fois, et l’odométrie a ±10 % d’erreur plus une petite dérive. Le robot ne connaît pas la salle : il ne voit que ses scans. Commande : `node bench_exploration.js`.

| Graine | Départ (x ; y ; cap) | Vitesse réelle | Fin | Couverture | Chocs | Ordres | Distance | Positions | Erreur de pose médiane / max | Arrivée à |
|---|---|---|---|---|---|---|---|---|---|---|
| 20 | 12.5 ; 2.7 ; -121° | × 0.91 | cartographie terminée, robot revenu au départ | 100 % | 0 | 167 | 39 m | 85 | 3.2 cm / 8.3 cm | 13.8 cm du départ |
| 21 | 5.9 ; 4.5 ; -78° | × 0.95 | cartographie terminée, robot revenu au départ | 100 % | 0 | 157 | 37 m | 81 | 1.8 cm / 3.7 cm | 4.2 cm du départ |
| 22 | 6.1 ; 6.6 ; 88° | × 0.81 | cartographie terminée, robot revenu au départ | 100 % | 0 | 187 | 39 m | 82 | 1.5 cm / 3.4 cm | 15.5 cm du départ |
| 23 | 1.7 ; 1.1 ; 48° | × 0.92 | cartographie terminée, robot revenu au départ | 100 % | 0 | 168 | 38 m | 83 | 2.0 cm / 3.0 cm | 12.3 cm du départ |
| 24 | 4.7 ; 9.2 ; -74° | × 0.84 | cartographie terminée, robot revenu au départ | 99 % | 0 | 223 | 43 m | 99 | 2.5 cm / 9.7 cm | 12.3 cm du départ |
| 25 | 5.8 ; 4.6 ; -59° | × 0.90 | cartographie terminée, robot revenu au départ | 100 % | 0 | 168 | 32 m | 84 | 1.9 cm / 3.6 cm | 3.6 cm du départ |
| 26 | 6.9 ; 7.2 ; 4° | × 0.80 | cartographie terminée, robot revenu au départ | 100 % | 0 | 149 | 30 m | 62 | 1.7 cm / 2.7 cm | 7.3 cm du départ |
| 27 | 2.6 ; 8.0 ; -93° | × 0.91 | cartographie terminée, robot revenu au départ | 100 % | 0 | 127 | 29 m | 61 | 2.4 cm / 4.1 cm | 13.2 cm du départ |
| 28 | 5.9 ; 6.9 ; 85° | × 1.10 | cartographie terminée, robot revenu au départ | 100 % | 0 | 153 | 28 m | 78 | 1.9 cm / 2.9 cm | 2.7 cm du départ |
| 29 | 3.6 ; 4.0 ; -55° | × 0.94 | cartographie terminée, robot revenu au départ | 100 % | 0 | 189 | 36 m | 92 | 1.6 cm / 3.1 cm | 16.6 cm du départ |
| 30 | 2.8 ; 1.9 ; -24° | × 1.15 | cartographie terminée, robot revenu au départ | 100 % | 0 | 157 | 31 m | 84 | 2.0 cm / 3.0 cm | 8.5 cm du départ |
| 31 | 1.1 ; 4.3 ; -56° | × 0.83 | cartographie terminée, robot revenu au départ | 100 % | 0 | 169 | 35 m | 72 | 2.0 cm / 2.8 cm | 8.8 cm du départ |

**Bilan.** Cartographie terminée dans 12/12 cas, avec retour au départ dans 12/12 cas. Couverture médiane 100 % (minimum 99 %). Chocs : 0. Erreur de pose médiane 2.0 cm, maximum 9.7 cm. 168 ordres et 36 m parcourus en médiane. La couverture est la part de la surface libre réelle que la carte marque libre.

## Contrôle du montage du lidar (premiers mouvements)

| Cas | Ordres avant l’arrêt | Chocs | Message |
|---|---|---|---|
| lidar tourné de 90° | 4 | 0 | arrêt : le robot avance à -91° de l’avant du lidar : le 0° du lidar n’est pas l’avant du robot. Mets le décalage d’angle à 90° de plus dans les options d’affichage, puis relance. |
| lidar tourné de 180° | 4 | 0 | arrêt : le robot avance à 176° de l’avant du lidar : le 0° du lidar n’est pas l’avant du robot. Mets le décalage d’angle à 183° de plus dans les options d’affichage, puis relance. |
| scan en miroir | 4 | 0 | arrêt : les rotations mesurées sont à l’envers de la commande : le scan est en miroir. Coche (ou décoche) « miroir » dans les options d’affichage, puis relance. |
