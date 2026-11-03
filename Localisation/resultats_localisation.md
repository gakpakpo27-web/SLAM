# Résultats du banc de localisation

Généré le 03/11/2026 10:39:57 · graine 7 · 5 tirages par niveau.

## Simulation (salle en L 13,5 × 9,8 m, 70 pieds de chaises, poses exactes)

Réussite : erreur ≤ 5 cm **et** ≤ 2° · réussite grossière : ≤ 20 cm et ≤ 5°. Fenêtre de recherche des méthodes globales : ±1.2 m, ±50°. 5 tirages par cas et par niveau (1 seul au niveau 0).

### Grid map

| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |
|---|---|---|---|---|---|---|
| Recherche exhaustive | 0.00 m / 0° | 100 % | 100 % | 0.0 cm | 0.00° | 217 ms |
| Recherche exhaustive | 0.10 m / 5° | 100 % | 100 % | 1.5 cm | 0.00° | 215 ms |
| Recherche exhaustive | 0.25 m / 10° | 100 % | 100 % | 2.0 cm | 0.00° | 214 ms |
| Recherche exhaustive | 0.50 m / 20° | 100 % | 100 % | 1.9 cm | 0.00° | 213 ms |
| Recherche exhaustive | 1.00 m / 45° | 100 % | 100 % | 2.0 cm | 0.00° | 211 ms |
| Descente de gradient (Gauss-Newton) | 0.00 m / 0° | 100 % | 100 % | 0.9 cm | 0.04° | 5 ms |
| Descente de gradient (Gauss-Newton) | 0.10 m / 5° | 100 % | 100 % | 0.9 cm | 0.04° | 3 ms |
| Descente de gradient (Gauss-Newton) | 0.25 m / 10° | 100 % | 100 % | 0.9 cm | 0.04° | 3 ms |
| Descente de gradient (Gauss-Newton) | 0.50 m / 20° | 70 % | 74 % | 1.2 cm | 0.07° | 2 ms |
| Descente de gradient (Gauss-Newton) | 1.00 m / 45° | 0 % | 0 % | 93.6 cm | 44.53° | 2 ms |
| Multi-résolution | 0.00 m / 0° | 100 % | 100 % | 0.0 cm | 0.00° | 15 ms |
| Multi-résolution | 0.10 m / 5° | 100 % | 100 % | 1.3 cm | 0.00° | 13 ms |
| Multi-résolution | 0.25 m / 10° | 100 % | 100 % | 1.6 cm | 0.00° | 13 ms |
| Multi-résolution | 0.50 m / 20° | 100 % | 100 % | 1.4 cm | 0.00° | 13 ms |
| Multi-résolution | 1.00 m / 45° | 100 % | 100 % | 1.4 cm | 0.00° | 12 ms |

### Carte vectorielle

| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |
|---|---|---|---|---|---|---|
| Recherche exhaustive | 0.00 m / 0° | 100 % | 100 % | 0.0 cm | 0.00° | 490 ms |
| Recherche exhaustive | 0.10 m / 5° | 100 % | 100 % | 0.9 cm | 0.00° | 491 ms |
| Recherche exhaustive | 0.25 m / 10° | 100 % | 100 % | 1.0 cm | 0.00° | 493 ms |
| Recherche exhaustive | 0.50 m / 20° | 100 % | 100 % | 1.1 cm | 0.00° | 481 ms |
| Recherche exhaustive | 1.00 m / 45° | 100 % | 100 % | 1.0 cm | 0.00° | 432 ms |
| Calcul direct (coins) + ICP | 0.00 m / 0° | 90 % (dont 10 % sans résultat) | 90 % | 0.3 cm | 0.03° | 2 ms |
| Calcul direct (coins) + ICP | 0.10 m / 5° | 90 % (dont 10 % sans résultat) | 90 % | 0.3 cm | 0.03° | 2 ms |
| Calcul direct (coins) + ICP | 0.25 m / 10° | 90 % (dont 10 % sans résultat) | 90 % | 0.3 cm | 0.03° | 2 ms |
| Calcul direct (coins) + ICP | 0.50 m / 20° | 90 % (dont 10 % sans résultat) | 90 % | 0.3 cm | 0.03° | 2 ms |
| Calcul direct (coins) + ICP | 1.00 m / 45° | 90 % (dont 10 % sans résultat) | 90 % | 0.3 cm | 0.03° | 2 ms |
| Descente de gradient (ICP point-segment) | 0.00 m / 0° | 100 % | 100 % | 0.3 cm | 0.02° | 2 ms |
| Descente de gradient (ICP point-segment) | 0.10 m / 5° | 100 % | 100 % | 0.4 cm | 0.03° | 2 ms |
| Descente de gradient (ICP point-segment) | 0.25 m / 10° | 100 % | 100 % | 0.4 cm | 0.03° | 2 ms |
| Descente de gradient (ICP point-segment) | 0.50 m / 20° | 8 % | 8 % | 18.9 cm | 12.95° | 1 ms |
| Descente de gradient (ICP point-segment) | 1.00 m / 45° | 0 % | 0 % | 85.8 cm | 43.80° | 1 ms |

### Entre scans

| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |
|---|---|---|---|---|---|---|
| Corrélation multi-résolution | 0.00 m / 0° | 100 % | 100 % | 0.4 cm | 0.04° | 64 ms |
| Corrélation multi-résolution | 0.10 m / 5° | 100 % | 100 % | 0.4 cm | 0.04° | 16 ms |
| Corrélation multi-résolution | 0.25 m / 10° | 100 % | 100 % | 0.4 cm | 0.04° | 15 ms |
| Corrélation multi-résolution | 0.50 m / 20° | 100 % | 100 % | 0.4 cm | 0.04° | 15 ms |
| Corrélation multi-résolution | 1.00 m / 45° | 100 % | 100 % | 0.4 cm | 0.04° | 14 ms |
| ICP point-à-ligne | 0.00 m / 0° | 100 % | 100 % | 0.3 cm | 0.04° | 10 ms |
| ICP point-à-ligne | 0.10 m / 5° | 100 % | 100 % | 0.5 cm | 0.06° | 8 ms |
| ICP point-à-ligne | 0.25 m / 10° | 100 % | 100 % | 0.5 cm | 0.07° | 6 ms |
| ICP point-à-ligne | 0.50 m / 20° | 98 % | 98 % | 0.5 cm | 0.05° | 6 ms |
| ICP point-à-ligne | 1.00 m / 45° | 4 % | 4 % | 74.8 cm | 43.08° | 4 ms |
| ICP point-à-point | 0.00 m / 0° | 100 % | 100 % | 0.3 cm | 0.06° | 8 ms |
| ICP point-à-point | 0.10 m / 5° | 98 % | 100 % | 0.5 cm | 0.14° | 9 ms |
| ICP point-à-point | 0.25 m / 10° | 52 % | 74 % | 1.1 cm | 0.28° | 10 ms |
| ICP point-à-point | 0.50 m / 20° | 0 % | 0 % | 26.3 cm | 17.27° | 5 ms |
| ICP point-à-point | 1.00 m / 45° | 0 % | 0 % | 90.1 cm | 44.62° | 4 ms |

## Incertitude des références réelles

Chaque scan réel a été recalé par ICP point-à-segment sur la carte des autres scans, depuis sa pose de référence perturbée de 3 cm et 1,5° (10 essais par scan). Écart à la référence : médiane 1.4 cm et 0.11°, 90e centile 2.3 cm et 0.55°. Les références réelles sont donc connues à environ ±3 cm et ±0.6° ; une erreur mesurée plus petite que cela n'est pas significative.

## Scans réels (14 scans, carte construite sans le scan testé)

Réussite : erreur ≤ 5 cm **et** ≤ 2° · réussite grossière : ≤ 20 cm et ≤ 5°. Fenêtre de recherche des méthodes globales : ±1.2 m, ±50°. 5 tirages par cas et par niveau (1 seul au niveau 0).

### Grid map

| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |
|---|---|---|---|---|---|---|
| Recherche exhaustive | 0.00 m / 0° | 93 % | 100 % | 0.0 cm | 0.00° | 200 ms |
| Recherche exhaustive | 0.10 m / 5° | 90 % | 100 % | 2.1 cm | 0.00° | 200 ms |
| Recherche exhaustive | 0.25 m / 10° | 91 % | 100 % | 2.3 cm | 0.00° | 200 ms |
| Recherche exhaustive | 0.50 m / 20° | 93 % | 100 % | 2.6 cm | 0.00° | 200 ms |
| Recherche exhaustive | 1.00 m / 45° | 91 % | 100 % | 2.1 cm | 0.00° | 193 ms |
| Descente de gradient (Gauss-Newton) | 0.00 m / 0° | 93 % | 100 % | 1.5 cm | 0.14° | 3 ms |
| Descente de gradient (Gauss-Newton) | 0.10 m / 5° | 94 % | 100 % | 1.5 cm | 0.15° | 3 ms |
| Descente de gradient (Gauss-Newton) | 0.25 m / 10° | 93 % | 97 % | 1.6 cm | 0.15° | 3 ms |
| Descente de gradient (Gauss-Newton) | 0.50 m / 20° | 11 % | 13 % | 54.2 cm | 18.80° | 3 ms |
| Descente de gradient (Gauss-Newton) | 1.00 m / 45° | 0 % | 0 % | 104.3 cm | 43.53° | 2 ms |
| Multi-résolution | 0.00 m / 0° | 100 % | 100 % | 2.5 cm | 0.00° | 17 ms |
| Multi-résolution | 0.10 m / 5° | 97 % | 100 % | 1.8 cm | 0.00° | 15 ms |
| Multi-résolution | 0.25 m / 10° | 96 % | 100 % | 1.9 cm | 0.00° | 15 ms |
| Multi-résolution | 0.50 m / 20° | 97 % | 100 % | 2.1 cm | 0.00° | 15 ms |
| Multi-résolution | 1.00 m / 45° | 96 % | 100 % | 1.9 cm | 0.00° | 14 ms |

### Carte vectorielle

| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |
|---|---|---|---|---|---|---|
| Recherche exhaustive | 0.00 m / 0° | 100 % | 100 % | 2.5 cm | 0.00° | 475 ms |
| Recherche exhaustive | 0.10 m / 5° | 100 % | 100 % | 2.0 cm | 0.00° | 485 ms |
| Recherche exhaustive | 0.25 m / 10° | 100 % | 100 % | 1.9 cm | 0.50° | 487 ms |
| Recherche exhaustive | 0.50 m / 20° | 100 % | 100 % | 1.6 cm | 0.00° | 474 ms |
| Recherche exhaustive | 1.00 m / 45° | 100 % | 100 % | 1.6 cm | 0.00° | 447 ms |
| Calcul direct (coins) + ICP | 0.00 m / 0° | 21 % (dont 57 % sans résultat) | 21 % | 240.1 cm | 42.65° | 3 ms |
| Calcul direct (coins) + ICP | 0.10 m / 5° | 21 % (dont 57 % sans résultat) | 21 % | 240.1 cm | 42.65° | 1 ms |
| Calcul direct (coins) + ICP | 0.25 m / 10° | 21 % (dont 57 % sans résultat) | 21 % | 240.1 cm | 42.65° | 1 ms |
| Calcul direct (coins) + ICP | 0.50 m / 20° | 21 % (dont 57 % sans résultat) | 21 % | 240.1 cm | 42.65° | 1 ms |
| Calcul direct (coins) + ICP | 1.00 m / 45° | 21 % (dont 57 % sans résultat) | 21 % | 240.1 cm | 42.65° | 1 ms |
| Descente de gradient (ICP point-segment) | 0.00 m / 0° | 100 % | 100 % | 1.4 cm | 0.13° | 1 ms |
| Descente de gradient (ICP point-segment) | 0.10 m / 5° | 100 % | 100 % | 1.4 cm | 0.11° | 1 ms |
| Descente de gradient (ICP point-segment) | 0.25 m / 10° | 89 % | 93 % | 1.7 cm | 0.11° | 1 ms |
| Descente de gradient (ICP point-segment) | 0.50 m / 20° | 3 % | 4 % | 49.3 cm | 17.14° | 1 ms |
| Descente de gradient (ICP point-segment) | 1.00 m / 45° | 0 % | 0 % | 101.1 cm | 44.43° | 1 ms |

### Entre scans

| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |
|---|---|---|---|---|---|---|
| Corrélation multi-résolution | 0.00 m / 0° | 100 % | 100 % | 0.8 cm | 0.25° | 32 ms |
| Corrélation multi-résolution | 0.10 m / 5° | 100 % | 100 % | 0.8 cm | 0.25° | 17 ms |
| Corrélation multi-résolution | 0.25 m / 10° | 100 % | 100 % | 0.8 cm | 0.25° | 18 ms |
| Corrélation multi-résolution | 0.50 m / 20° | 100 % | 100 % | 0.8 cm | 0.25° | 18 ms |
| Corrélation multi-résolution | 1.00 m / 45° | 100 % | 100 % | 0.8 cm | 0.25° | 17 ms |
| ICP point-à-ligne | 0.00 m / 0° | 100 % | 100 % | 0.9 cm | 0.13° | 5 ms |
| ICP point-à-ligne | 0.10 m / 5° | 100 % | 100 % | 0.9 cm | 0.13° | 5 ms |
| ICP point-à-ligne | 0.25 m / 10° | 98 % | 98 % | 0.9 cm | 0.13° | 5 ms |
| ICP point-à-ligne | 0.50 m / 20° | 42 % | 42 % | 18.0 cm | 9.13° | 5 ms |
| ICP point-à-ligne | 1.00 m / 45° | 0 % | 0 % | 96.7 cm | 42.19° | 5 ms |
| ICP point-à-point | 0.00 m / 0° | 100 % | 100 % | 1.3 cm | 0.23° | 5 ms |
| ICP point-à-point | 0.10 m / 5° | 95 % | 100 % | 1.1 cm | 0.35° | 5 ms |
| ICP point-à-point | 0.25 m / 10° | 64 % | 71 % | 1.8 cm | 0.77° | 7 ms |
| ICP point-à-point | 0.50 m / 20° | 0 % | 0 % | 42.2 cm | 18.10° | 5 ms |
| ICP point-à-point | 1.00 m / 45° | 0 % | 0 % | 90.9 cm | 44.67° | 4 ms |

