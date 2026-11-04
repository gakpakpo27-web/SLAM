# Superposition : rotations du robot et coupures de liaison

Commande : `node bench_rotation_coupure.js`. Une position est « mal superposée » si elle est à plus de 10 cm ou 3° de sa vraie place, dans le repère du premier scan.

## Robot tourné à la main (30 à 120° entre deux scans, sans commande)

6 parcours de 60 pas.

| Cartographe | Positions enregistrées | Mal superposées | Pire écart | Scans « perdu » |
|---|---|---|---|---|
| Avant (recherche à ±25° de la prédiction, tout scan posé) | 63 | 36 | 283 cm | 307 |
| Maintenant (toutes orientations, pas de scan douteux, pas de doublon) | 57 | 1 | 15 cm | 0 |

## Coupure de liaison (2 s sans scan, robot déplacé de 0,3 à 0,8 m et tourné de 20 à 120°)

8 parcours, 5 coupures chacun.

| | Positions enregistrées | Mal superposées | Pire écart |
|---|---|---|---|
| Coupure ignorée | 96 | 17 | 53 cm |
| Coupure détectée : on attend que le robot se retrouve | 84 | 0 | 3 cm |
