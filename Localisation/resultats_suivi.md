# Robot en direct : localisation globale et suivi

## 1. Localisation globale sur les scans réels (sans pose de départ)

Carte de chaque essai = les 13 autres scans. Réussite : ≤ 5 cm et ≤ 2°.

| Scan | Erreur position | Erreur angle | Points sur la carte | Confiance | 2e hypothèse / 1re | Bonne pose parmi les hypothèses | Temps |
|---|---|---|---|---|---|---|---|
| pos1 | 0.7 cm | 0.11° | 92 % | sûr | 0.69 | oui (n° 1) | 328 ms |
| pos2 | 1499.8 cm | 178.70° | 48 % | ambigu | 0.91 | non | 256 ms |
| pos3 | 1046.6 cm | 89.84° | 52 % | ambigu | 0.83 | oui (n° 3) | 228 ms |
| pos4 | 1.9 cm | 0.08° | 50 % | sûr | 0.64 | oui (n° 1) | 163 ms |
| pos5 | 0.4 cm | 0.08° | 49 % | sûr | 0.66 | oui (n° 1) | 225 ms |
| pos6 | 2.3 cm | 0.40° | 54 % | sûr | 0.54 | oui (n° 1) | 175 ms |
| pos7 | 0.7 cm | 0.03° | 53 % | sûr | 0.55 | oui (n° 1) | 278 ms |
| pos8 | 3.1 cm | 0.15° | 43 % | ambigu | 0.85 | oui (n° 1) | 244 ms |
| pos9 | 1.9 cm | 0.06° | 46 % | sûr | 0.74 | oui (n° 1) | 235 ms |
| pos10 | 1.2 cm | 0.69° | 37 % | sûr | 0.79 | oui (n° 1) | 223 ms |
| pos11 | 1.4 cm | 0.36° | 37 % | sûr | 0.73 | oui (n° 1) | 202 ms |
| pos12 | 2.0 cm | 0.09° | 64 % | sûr | 0.72 | oui (n° 1) | 167 ms |
| pos13 | 2.0 cm | 0.58° | 61 % | sûr | 0.69 | oui (n° 1) | 222 ms |
| pos14 | 1.6 cm | 0.55° | 45 % | sûr | 0.70 | oui (n° 1) | 219 ms |

**Réussite : 12/14** · classés « sûr » : 11, dont justes 11 · bonne pose parmi les hypothèses : 13/14 · temps médian 225 ms

## 2. Suivi en simulation (poses exactes connues)

Salle simulée, carte de 9 scans, trajet de 264 scans (pas de 15 cm, rotations de 15°). Odométrie : déplacement commandé avec 10 % d'erreur, plus un biais de 1 cm et 0,5° par pas.

| Scénario | Scans justes (≤ 5 cm, 2°) | Erreur médiane | Temps médian (suivi) | Recherches globales | Scans pour se relocaliser après le kidnapping |
|---|---|---|---|---|---|
| Avec odométrie | 264/264 (100 %) | 0.3 cm, 0.03° | 6 ms | 1 | – |
| Sans odométrie (pose précédente seule) | 264/264 (100 %) | 0.3 cm, 0.03° | 5 ms | 1 | – |
| Salle encombrée : 25 objets absents de la carte | 264/264 (100 %) | 0.4 cm, 0.04° | 5 ms | 1 | – |
| Robot kidnappé à mi-parcours | 264/264 (100 %) | 0.3 cm, 0.03° | 7 ms | 2 | 1 |
