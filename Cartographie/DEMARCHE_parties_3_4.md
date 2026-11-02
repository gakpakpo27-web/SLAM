# Parties 3 et 4 : grid map 2D et carte vectorielle

**Outils :** `lidar_viewer.html`, onglets **Grid map** et **Segments**.
**Calculs :** `lidar_core.js` (fonctions pures, sans dépendance).
**Tests :** `node test_lidar_core.js`, ou bouton *Lancer les tests* dans l'onglet Segments : 74 tests au total (parties 3 à 5), tous réussis.

Les deux outils prennent en entrée les scans de la partie 1 avec leur pose, fixée dans la partie 2 (superposition).

---

## 0. Hypothèses communes

| Hypothèse | Justification |
|---|---|
| **Carte en 2D.** Le lidar balaie un plan horizontal à hauteur fixe. | Le LD06 est un lidar 2D. Tout ce qui est plus haut ou plus bas que le plan est invisible (plateaux de table, objets au sol). |
| **La pose de chaque scan est connue et fiable.** | Elle vient de la superposition de la partie 2 (manuelle + ICP). Une erreur de pose se retrouve telle quelle dans la carte. |
| **Le robot est immobile pendant un scan.** | Les scans sont pris à l'arrêt. Sinon, il faudrait corriger la déformation due au mouvement pendant le tour. |
| **Angles en sens trigonométrique, distances en mètres.** | La librairie LD06 convertit déjà l'angle (`360° − angle physique`). Une seule convention est utilisée partout. |
| **Le lidar est au centre du robot.** | Sinon, il faut ajouter le décalage capteur → robot dans la pose. |

La conversion polaire → carte est la même pour les deux parties :
`P_local = (r cos α, r sin α)`, puis `P_global = R(θ) · P_local + (x, y)`.
Les points hors de `[distance min, distance max]` ou sous l'intensité minimale sont supprimés avant tout traitement. Les points isolés sont aussi retirés : un point sans voisin proche parmi les 3 points angulairement voisins de chaque côté est considéré comme du bruit.

---

## 3. Construction d'une grid map 2D

### 3.1 Définition de la grille

**Choix :**
- **Résolution par défaut : 5 cm.** C'est un compromis entre précision et mémoire (voir ci-dessous).
- **Taille et origine calculées automatiquement.** La grille couvre la boîte englobante du scan et du robot, plus une marge de 0,5 m. L'origine `(x0, y0)` est le coin bas-gauche.
- **Stockage : `Uint8Array` de W × H octets**, un octet par cellule (comme recommandé pour un microcontrôleur). La ligne `j = 0` est en bas, à y minimum.
- **Valeur d'une cellule :**

  | Valeur | Signification |
  |---|---|
  | 0 | libre certain |
  | 255 | occupé certain |
  | 127 | inconnu (état initial par défaut) |

  Les états initiaux « tout libre » et « tout occupé » restent disponibles.
- **Seuils de lecture :** une cellule est libre si sa valeur est ≤ 85, occupée si elle est ≥ 170, inconnue entre les deux.
- **Conversion monde → cellule :** `i = floor((x − x0) / rés)`, `j = floor((y − y0) / rés)`. On utilise `floor` et pas l'arrondi, pour que les coordonnées négatives tombent dans la bonne cellule (vérifié par un test).

**Ordre de grandeur :**
- La salle réelle (6 scans Teleplot) donne 321 × 226 cellules, soit 72 546 octets (71 Ko).
- À 1 cm de résolution, il faudrait 25 fois plus de mémoire (1,8 Mo), pour une précision que le LD06 ne fournit pas : son bruit est de l'ordre du centimètre.

### 3.2 Mise à jour : cellules traversées et cellule d'impact

Pour chaque point du scan :

1. **Cellule d'impact.** C'est la cellule contenant le point. Elle est marquée « occupée ».
2. **Cellules traversées.** On trace le segment robot → point avec l'algorithme de **Bresenham**, en arithmétique entière, dans les 8 directions. Toutes les cellules traversées sauf la cellule d'impact sont marquées « libres ».

**Choix du modèle (au choix dans l'outil) :**
- **Probabiliste (par défaut).**
  - Une cellule libre perd 45 et une cellule d'impact gagne 70, avec saturation dans [0, 255].
  - Cette addition saturée est l'équivalent quantifié d'un filtre bayésien en *log-odds* : additionner des log-odds revient à combiner des observations indépendantes.
  - Les valeurs 45 et 70 sont choisies pour qu'**une seule observation suffise** à franchir les seuils (127 − 45 = 82 ≤ 85 et 127 + 70 = 197 ≥ 170). Un premier scan donne donc immédiatement une carte lisible.
  - L'impact pèse plus que le passage libre : un obstacle détecté est une information plus sûre qu'une absence de retour.
- **Binaire.** Affectation directe (0 ou 255), sans mémoire : c'est la dernière observation qui gagne.

**Choix d'implémentation importants :**
- **Une cellule n'est mise à jour qu'une fois par scan.** Les cellules libres et les cellules d'impact sont d'abord rassemblées dans deux ensembles, puis appliquées. Sans cela, les cellules proches du robot, traversées par des centaines de faisceaux, deviendraient « libres certaines » en un seul scan, alors qu'il n'y a qu'une seule observation.
- **L'impact est prioritaire sur le passage libre dans un même scan.** Quand un faisceau rase un mur, il peut traverser la cellule d'impact d'un faisceau voisin. Sans cette règle, le mur se troue. Un test vérifie ce cas.
- **Point hors de la carte.** Le rayon est tout de même tracé, et ses cellules situées dans la carte sont libérées. Le nombre de points hors carte est affiché.

### 3.3 Outils réalisés

| Outil | Entrée | Sortie | Fonction |
|---|---|---|---|
| **1 · Scan + pose → carte** | un scan et sa pose | une nouvelle grille, dimensionnée sur le scan | `scanToGrid` |
| **2 · Carte + scan + pose → carte mise à jour** | une grille existante (en mémoire ou chargée depuis un `.json`), un scan et sa pose | la grille mise à jour | `updateGrid` |

Pour l'outil 2 :
- **Agrandissement automatique.** Si le scan dépasse la carte, celle-ci est agrandie (`ensureCovers`). Le contenu est recopié au même endroit, sans rééchantillonnage, car l'agrandissement se fait par nombre entier de cellules.
- **Annulation.** La carte précédente est conservée et la mise à jour peut être annulée.

**Construire avec tous les scans visibles** enchaîne les deux outils : on crée une grille couvrant tous les scans, puis on y intègre chaque scan.

**Formats de sortie :**
- `.json` : rechargeable dans l'outil 2.
- `.pgm` + `.yaml` : format `map_server` de ROS (blanc = libre).
- `.png` : pour le rapport.

### 3.4 Difficultés rencontrées

**1. Stries de cellules inconnues loin du robot.**
- *Cause :* le LD06 mesure environ tous les 0,8°. À 6 m, deux faisceaux voisins sont donc écartés de 8 cm, plus qu'une cellule de 5 cm. Des cellules libres ne sont traversées par aucun faisceau et restent inconnues : la carte est striée « en rayons de soleil ».
- *Solution (option activée par défaut) :* si deux points successifs (ordre angulaire) sont proches, ils sont sur la même surface. Le triangle robot – p(k) – p(k+1) est alors vu libre, et on le remplit.
  - Si les deux points sont trop éloignés (discontinuité, par exemple le bord d'un poteau), on ne remplit pas : la zone d'ombre derrière l'obstacle reste inconnue (testé).
  - Le bord du triangle est reculé d'une cellule avant les impacts, pour ne pas « trouer » le mur entre deux impacts (testé).

**2. Nuances de gris dans les zones libres.**
- Ce n'est pas un défaut : une zone vue par un seul scan vaut 82 (gris clair), une zone vue par plusieurs scans tend vers 0 (blanc).
- L'option *Affichage seuillé* montre les trois états (libre, occupé, inconnu).

**3. Cohérence entre scans.** Une erreur de superposition de 2–3 cm élargit les murs d'une cellule. Une erreur plus grande les dédouble. La qualité de la grille dépend directement de la partie 2.

### 3.5 Limites

- **Résolution fixe.** Il faut choisir entre précision et mémoire, et une carte grande et fine coûte cher. Une limite de 40 millions de cellules est imposée.
- **Obstacles hors du plan non vus.** Plateaux de table, objets bas, vitres et surfaces noires (absorbent l'infrarouge) sont invisibles. Une vitre apparaît comme du libre.
- **Hypothèse d'indépendance des cellules.** Elle est fausse en pratique : les cellules d'un même mur sont liées.
- **Bord d'un objet.** Un point qui tombe sur le bord d'un objet occupe une cellule entière.
- **Objets mobiles.** Une personne qui passe laisse une trace occupée, que les scans suivants effacent progressivement, grâce à la mise à jour en log-odds.
- **Non réalisé :**
  - nettoyage de la carte (décroissance temporelle, lissage spatial) ;
  - recalage automatique d'un nouveau scan sur la grille (scan matching). Ici, la pose vient de la partie 2.

---

## 4. Extraction et manipulation de segments

### 4.1 Nuage de points → segments

Méthode **split-and-merge**, sur les points d'un scan dans l'ordre angulaire :

1. **Segmentation en groupes.**
   - Un nouveau groupe commence quand deux points successifs sont séparés de plus de `0,10 m + 3 % × distance`.
   - Le seuil augmente avec la distance, parce que les points s'espacent quand on s'éloigne du robot.
   - Le parcours commence après la plus grande rupture, pour ne pas couper un mur en deux à l'angle 0°/360°.
2. **Découpe récursive (Iterative End Point Fit).**
   - On trace la corde entre le premier et le dernier point du groupe.
   - Si le point le plus éloigné de la corde est à plus de 4 cm, on coupe en ce point et on recommence sur chaque moitié.
   - Le seuil de 4 cm est environ 3 fois le bruit du LD06 : en dessous, on découperait le bruit ; au-dessus, on raterait des coins peu marqués.
3. **Ajustement de chaque morceau par moindres carrés totaux (ACP).**
   - La droite minimise les distances *perpendiculaires*, et pas les écarts en y comme une régression classique (qui échoue sur les murs verticaux).
   - Les extrémités sont les projections du premier et du dernier point sur cette droite.
4. **Fusion.** Les morceaux voisins colinéaires sont fusionnés (voir 4.3).
5. **Filtrage.** On ne garde que les segments d'au moins 6 points et 0,25 m.

**Avec plusieurs scans**, les segments de tous les scans sont regroupés, puis fusionnés deux à deux tant que deux segments sont « proches ». Le résultat est une carte vectorielle globale, exportée au format du cours (`x1,y1,x2,y2` par ligne).

**Résultats :**
- **Pièce simulée** (bruit ±1,5 cm) : les 4 murs sont retrouvés, orientés à moins de 1° des axes, et le mur contrôlé (celui du bas) est placé à moins de 1 cm de sa position réelle. Les faces d'un poteau de 20 cm sont aussi détectées.
- **Salle réelle (6 scans Teleplot) :**

  | Étape | Nombre de segments |
  |---|---|
  | Extraction, scan par scan | 171 |
  | Après fusion | 44 |
  | Après nettoyage (moins de 0,5 m supprimés) | 22 |

### 4.2 Angle entre deux segments

On renvoie deux valeurs :
- **Angle entre les droites, dans [0°, 90°].** L'orientation des segments est ignorée : un mur parcouru dans un sens ou dans l'autre reste le même mur. C'est cet angle qui sert pour la proximité.
- **Angle orienté de A vers B, dans ]−180°, 180°].** Utile quand le sens compte.

Le calcul utilise `atan2` sur les vecteurs directeurs, ce qui est stable pour tous les cas, y compris les segments verticaux.

### 4.3 Segments « proches »

Trois critères, tous réglables :

| Critère | Défaut | Mesure |
|---|---|---|
| Angulaire | ≤ 5° | angle entre les droites |
| Latéral | ≤ 8 cm | distance des extrémités du segment le plus court à la droite du plus long (on garde la plus grande des deux) |
| Longitudinal | ≤ 15 cm | trou entre les projections des deux segments sur leur direction moyenne (0 s'ils se chevauchent) |

- **Pourquoi le critère longitudinal ?** Sans lui, deux murs alignés de part et d'autre d'une porte seraient fusionnés à travers la porte.
- **Pourquoi la droite du plus long ?** Elle est la plus fiable. Une première version prolongeait aussi la droite du segment court : pour un petit morceau bruité incliné de 2°, l'écart atteignait 16 cm au milieu d'un mur de 8 m, et les deux morceaux d'un même mur n'étaient pas fusionnés (corrigé et testé, voir `DEMARCHE_partie_5.md`).
- **Explication du résultat.** L'outil affiche la valeur de chaque critère, pour comprendre pourquoi deux segments sont, ou ne sont pas, proches.

### 4.4 Fusion de deux segments

1. **Direction.** C'est la moyenne des directions, **pondérée par la longueur** et calculée sur l'angle doublé : ainsi, deux segments de sens opposés ne s'annulent pas.
2. **Position.** La droite passe par le barycentre des milieux, lui aussi pondéré par la longueur. Un long mur fiable n'est donc presque pas déplacé par un petit morceau.
3. **Extrémités.** Ce sont les projections extrêmes des 4 extrémités sur cette droite : le segment fusionné couvre les deux segments.
4. **Sens.** Le résultat garde le sens du premier segment.

### 4.5 Intersection de deux segments

On résout `A1 + t·(A2 − A1) = B1 + u·(B2 − B1)` avec des produits vectoriels. Résultats possibles :

| Type | Signification |
|---|---|
| `point` | Le point est sur les deux segments. `t` et `u` donnent sa position (0 = premier point, 1 = second). |
| `lines` | Les droites se coupent, mais hors d'au moins un segment. L'outil précise lequel. |
| `parallel` | Parallèles distincts. |
| `collinear` | Même droite. L'outil donne la partie commune (segment, point, ou rien). |

### 4.6 Découpe d'un segment par un autre

Règles appliquées (fonction `cutSegment(cible, coupeur)`) :

1. **Intersection obligatoire.** Il faut une intersection située sur les deux segments :
   - sur le coupeur, ses extrémités sont incluses : un coupeur qui touche la cible en « T » la coupe ;
   - sur la cible, elle doit être **strictement à l'intérieur**, à plus de EPS de ses deux extrémités.
2. **Intersection à une extrémité de la cible :** rien ne change.
3. **Segments colinéaires ou parallèles :** rien ne change.
4. **Côté conservé :** le plus long. Si les deux côtés sont **égaux à EPS près**, on garde celui qui contient le **premier point** (x1, y1) de la cible.

### 4.7 Tolérance numérique

- **EPS = 10⁻⁶ m (1 µm), exprimé en distance.**
  - C'est bien plus que les erreurs d'arrondi des doubles (environ 10⁻¹⁵ m sur 10 m).
  - C'est bien moins que la précision du LD06 (1 cm) : la tolérance ne cache aucun cas réel.
  - Elle est exprimée en mètres, et pas sur le paramètre t. Ainsi, « à une extrémité » signifie la même chose pour un segment de 10 cm et pour un segment de 10 m.
- **EPS_SIN = 10⁻⁹.** Deux directions sont parallèles si |sin(angle)| ≤ 10⁻⁹.
  - Ce test est séparé du précédent, pour que deux segments presque parallèles (0,01°) qui se croisent bien soient quand même coupés (testé).

**Cas limites testés (22 tests sur la découpe) :**

| Cas | Résultat attendu |
|---|---|
| Croisement à 3 m d'une cible de 10 m | côté du second point (7 m) |
| Croisement à 7 m | côté du premier point (7 m) |
| Croisement oblique à 45° | découpe au bon point |
| Côtés exactement égaux | côté du premier point |
| Côtés égaux à 10⁻⁷ m près (< EPS), dans les deux sens | côté du premier point |
| Côtés différents de 10⁻⁵ m (> EPS) | vraiment le plus long |
| Même cible parcourue dans l'autre sens | le « premier point » change, donc le côté gardé aussi |
| Intersection exactement au premier point, puis au second | rien ne change |
| Intersection à 5·10⁻⁷ m d'une extrémité (< EPS) | considérée à l'extrémité : rien ne change |
| Intersection à 5·10⁻⁶ m d'une extrémité (> EPS) | découpe |
| Coupeur qui touche la cible par son extrémité (T) | découpe |
| Coupeur qui s'arrête à 1 mm de la cible | rien ne change |
| Coupeur qui s'arrête à 5·10⁻⁷ m de la cible | découpe (contact à EPS près) |
| Droites sécantes hors de la cible | rien ne change |
| Colinéaires qui se chevauchent / bout à bout | rien ne change |
| Parallèles distincts | rien ne change |
| Presque parallèles (0,01°) qui se croisent | découpe |
| Segment de longueur nulle | rien ne change |

Dans l'outil, le mode **Tracer un segment** permet de rejouer ces cas à la main. La partie supprimée par une découpe s'affiche en pointillés rouges, et le premier point de chaque segment est marqué d'un disque plein.

### 4.8 Difficultés et limites

- **Réglage des seuils.** Les seuils de rupture et de découpe dépendent de la distance et du bruit. Une seule valeur ne convient pas à tout : à 7 m, le bruit du LD06 (plusieurs cm) fait sortir des murs en plusieurs morceaux légèrement inclinés.
- **Petits segments parasites.** Aux coins, aux bords de porte et sur les pieds de meubles, l'extraction produit de petits segments de 25 à 50 cm. Le bouton *Nettoyer* les supprime en dessous d'une longueur choisie, mais cela supprime aussi de vrais petits objets.
- **Fusion gloutonne.** Deux segments sont fusionnés dès qu'ils sont proches, dans l'ordre de la liste. Le résultat peut dépendre de cet ordre, et une chaîne de petites fusions peut faire tourner légèrement un long mur.
- **Arrondis non gérés.** Les cercles et arcs (poteaux ronds, poubelles) sont approchés par des segments courts. L'ajustement de cercle du cours n'est pas réalisé.
- **Pas de gestion des absences.** Les fonctions d'intersection et de découpe en sont la base : un faisceau qui traverse un segment de la carte signale un obstacle disparu. Mais la mise à jour automatique de la carte vectorielle au fil des scans n'est pas faite.
- **Qualité dépendante de la pose.** Comme pour la grille, tout dépend de la superposition de la partie 2.
