# Traitement de la carte

## Étapes

| Étape | Résultat | Temps |
|---|---|---|
| Recalage global des poses | 28 corrections acceptées sur 52 essais | 12078 ms |
| Prétraitement | 923 points écartés sur 19077 : hors portée 228, intensité 0, isolés 566, mixtes 129 | 22 ms |
| Grid map enrichie (impacts, observations, horodatage) | 292 × 260 cellules, 14 scans | 235 ms |
| Obstacles mobiles (persistance) | 2556 cellules « traces » (impact dans moins d'un tiers des observations) rendues libres, dont 0 encore occupées et 455 inconnues | 2 ms |
| Taches isolées | 63 cellules de bruit retirées, 91 petits obstacles confirmés gardés | 5 ms |
| Lissage (majoritaire) | 404 cellules inconnues tranchées | 4 ms |
| Fermeture des murs | 235 trous bouchés | 10 ms |
| Obstacles (composantes connexes) | 103 : 8 structures, 42 objets, 53 petits | 1 ms |
| Carte vectorielle + obstacles circulaires | 46 segments ; 842 cercles observés -> 188 obstacles circulaires suivis (33 de confiance ≥ 3, 0 mobiles), 94 écartés car posés sur un mur | 191 ms |
| Régularisation de la carte vectorielle | 46 → 43 segments ; 26 redressés (axes 0.6° / 90.6°), 13 alignés, 3 fusions, 27 coins raccordés, 5 dépassements rognés | 6 ms |
| Carte de coût | robot 15 cm + marge 35 cm ; 106.4 m² accessibles au centre du robot | 8 ms |

## Qualité de la grid map

| Mesure | Brute | Traitée |
|---|---|---|
| Points des scans sur un obstacle (≤ 1,5 cellule) | 56.9 % | 56.6 % |
| Distance moyenne point → obstacle | 16.3 cm | 16.2 cm |
| Cellules occupées | 1340 | 1554 |
| Surface libre | 114.8 m² | 116.6 m² |
| Netteté (cellules observées tranchées) | 98.0 % | 99.9 % |
| Cellules en conflit (persistance 25–75 %) | 1207 | 1207 |

## Recalage des poses

| Passe | Scan | dx (cm) | dy (cm) | dθ (°) | Score avant | Score après | Acceptée |
|---|---|---|---|---|---|---|---|
| 1 | pos5 | -2.3 | -0.5 | 0.15 | 0.230 | 0.233 | oui |
| 1 | pos6 | -1.9 | -0.7 | 0.11 | 0.354 | 0.364 | oui |
| 1 | pos12 | -0.0 | -0.6 | 0.15 | 0.449 | 0.451 | oui |
| 1 | pos11 | -1.4 | 0.3 | -0.29 | 0.267 | 0.271 | oui |
| 1 | pos10 | -0.3 | -0.4 | 0.39 | 0.394 | 0.405 | oui |
| 1 | pos4 | -1.0 | -1.0 | -0.10 | 0.389 | 0.393 | oui |
| 1 | pos7 | -1.5 | -0.4 | -0.22 | 0.440 | 0.452 | oui |
| 1 | pos2 | -0.4 | 0.6 | 0.48 | 0.251 | 0.259 | oui |
| 1 | pos9 | 0.2 | -2.0 | 0.01 | 0.363 | 0.371 | oui |
| 1 | pos8 | 1.3 | -3.1 | 0.22 | 0.371 | 0.395 | oui |
| 1 | pos3 | 0.8 | 4.2 | 1.03 | 0.351 | 0.367 | oui |
| 1 | pos13 | -1.4 | -2.5 | -0.41 | 0.498 | 0.515 | oui |
| 1 | pos14 | -1.3 | 0.0 | 0.49 | 0.292 | 0.311 | oui |
| 2 | pos2 | -0.6 | -1.9 | -0.35 | 0.286 | 0.292 | oui |
| 2 | pos3 | -1.9 | -4.7 | -1.14 | 0.349 | 0.380 | oui |
| 2 | pos4 | 1.1 | 0.7 | 0.03 | 0.397 | 0.403 | oui |
| 2 | pos5 | 0.7 | 0.2 | -0.04 | 0.386 | 0.390 | oui |
| 2 | pos6 | 2.7 | 0.1 | 0.01 | 0.376 | 0.387 | oui |
| 2 | pos7 | 0.8 | 0.7 | 0.11 | 0.390 | 0.394 | oui |
| 2 | pos8 | 0.7 | 1.8 | -0.37 | 0.331 | 0.350 | oui |
| 2 | pos9 | 0.8 | 1.4 | -0.00 | 0.347 | 0.353 | oui |
| 2 | pos10 | -0.6 | -0.3 | 0.25 | 0.375 | 0.373 | non |
| 2 | pos11 | -0.2 | -0.3 | 0.12 | 0.387 | 0.385 | non |
| 2 | pos12 | 1.3 | 0.8 | -0.23 | 0.485 | 0.494 | oui |
| 2 | pos13 | 0.3 | 1.0 | -0.03 | 0.533 | 0.539 | oui |
| 2 | pos14 | -0.2 | 0.7 | -0.03 | 0.336 | 0.336 | oui |
| 3 | pos2 | 0.2 | 1.0 | 0.11 | 0.304 | 0.307 | oui |
| 3 | pos3 | 0.2 | 0.2 | -0.01 | 0.380 | 0.381 | oui |
| 3 | pos4 | 0.5 | 0.4 | 0.04 | 0.403 | 0.405 | oui |
| 3 | pos5 | 0.6 | 0.2 | -0.00 | 0.387 | 0.389 | oui |
| 3 | pos6 | -0.1 | 0.3 | -0.06 | 0.386 | 0.387 | oui |
| 3 | pos7 | 0.4 | 0.5 | -0.03 | 0.391 | 0.393 | oui |
| 3 | pos8 | 0.4 | 0.7 | 0.22 | 0.352 | 0.351 | non |
| 3 | pos9 | 0.2 | 0.3 | -0.01 | 0.356 | 0.356 | oui |
| 3 | pos10 | -0.0 | -0.5 | 0.00 | 0.375 | 0.375 | oui |
| 3 | pos11 | -0.1 | -0.3 | 0.15 | 0.386 | 0.385 | non |
| 3 | pos12 | 0.0 | -0.0 | 0.02 | 0.496 | 0.497 | oui |
| 3 | pos13 | 0.1 | 0.3 | 0.01 | 0.541 | 0.541 | oui |
| 3 | pos14 | 0.1 | -0.1 | 0.02 | 0.339 | 0.338 | oui |
| 4 | pos2 | 0.0 | -0.0 | -0.03 | 0.306 | 0.306 | oui |
| 4 | pos3 | 0.1 | 0.1 | 0.00 | 0.381 | 0.381 | oui |
| 4 | pos4 | 0.1 | 0.1 | 0.00 | 0.405 | 0.405 | oui |
| 4 | pos5 | 0.3 | 0.1 | -0.04 | 0.391 | 0.392 | oui |
| 4 | pos6 | 1.8 | 0.2 | 0.13 | 0.387 | 0.385 | non |
| 4 | pos7 | 0.1 | 0.0 | 0.01 | 0.394 | 0.394 | oui |
| 4 | pos8 | 0.5 | 0.6 | 0.21 | 0.352 | 0.351 | non |
| 4 | pos9 | 0.0 | 0.0 | 0.00 | 0.356 | 0.355 | oui |
| 4 | pos10 | 0.1 | 0.0 | 0.01 | 0.375 | 0.375 | oui |
| 4 | pos11 | -0.1 | -0.2 | 0.16 | 0.386 | 0.384 | non |
| 4 | pos12 | 0.0 | -0.2 | 0.01 | 0.497 | 0.497 | oui |
| 4 | pos13 | 0.0 | -0.0 | -0.00 | 0.541 | 0.541 | oui |
| 4 | pos14 | 0.0 | -0.1 | 0.01 | 0.340 | 0.340 | oui |

## Obstacles de la grille

| Classe | Nombre | Surface |
|---|---|---|
| structure | 8 | 2.71 m² |
| objet | 42 | 1.00 m² |
| petit | 53 | 0.17 m² |
