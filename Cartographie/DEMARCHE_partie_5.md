# Partie 5 : construction d'une carte vectorielle

**Outil :** `lidar_viewer.html`, onglet **Vectorielle** (intégré au même outil que la grid map).
**Calculs :** `lidar_core.js` (`createVectorMap`, `updateVectorMap`, `computeCorners`, `vectorMapToText`, `vectorMapFromText`).
**Tests :** 13 tests dans `test_lidar_core.js` (groupe « Carte vectorielle »), plus 1 test de proximité ajouté pour la correction décrite à l'étape 3. Au total, 74 tests réussis.

La carte vectorielle réutilise les fonctions de la partie 4 :

| Fonction (partie 4) | Utilisée pour |
|---|---|
| `extractSegments` | extraction |
| `areClose` | association |
| `mergeSegments` | fusion |
| `intersect` | gestion des absences |
| `angleBetween` | faisceaux rasants, coins |

Les hypothèses communes (carte 2D, poses connues grâce à la partie 2, robot immobile pendant un scan) sont celles décrites dans `DEMARCHE_parties_3_4.md`.

---

## Étape 1 : définir ce que l'on représente

**Choix :** l'environnement est une salle intérieure, structurée. On représente donc les murs et les faces des meubles par des **segments**. On ajoute aussi des **coins** : des points orientés (x, y, θ), calculés à partir des segments.

**Les cercles et arcs ne sont pas traités.** Les obstacles ronds de la salle (pieds de chaises) sont trop petits pour le LD06 : 1 à 3 points chacun. Ils sont filtrés comme du bruit.

**Structure d'un segment (« feature ») :**

| Champ | Rôle |
|---|---|
| `id` | identifiant unique |
| `x1, y1, x2, y2` | extrémités (m, repère de la carte) |
| `conf` | **compteur de confiance** : +1 quand le segment est revu, −1 quand il est traversé, plafonné à 10 |
| `nObs` | nombre de scans où il a été observé |
| `firstSeen`, `lastSeen` | numéro du premier et du dernier scan où il a été vu (**horodatage**, utilisé pour la durée de vie) |
| `n` | nombre de points lidar qui l'ont produit |

**La carte :** une liste de segments et une liste de coins. La carte de départ est vide, ou chargée depuis un fichier de murs connus (format simple `x1,y1,x2,y2`, comme dans le document du cours).

**Export texte :** `x1,y1,x2,y2,confiance,observations,premier_scan,dernier_scan` par ligne, puis les coins (`C,x,y,theta`). Les 4 premières colonnes respectent le format du cours, donc le fichier reste lisible par un programme qui n'attend que les segments.

## Étape 2 : acquisition et prétraitement

C'est la même chaîne que pour la grid map :
1. conversion polaire → cartésien → repère de la carte avec la pose du scan ;
2. suppression des points hors de [distance min, distance max] et sous l'intensité minimale ;
3. suppression des points isolés.

Les points restent **dans l'ordre angulaire** autour du robot, ce qui est indispensable pour l'étape suivante.

## Étape 3 : extraction des primitives

On utilise `extractSegments` de la partie 4, sans modification :
1. **Segmentation.** Un nouveau groupe commence quand l'écart entre deux points successifs dépasse `0,10 m + 3 % × distance`.
2. **Découpe récursive** au point le plus éloigné de la corde (seuil 4 cm).
3. **Ajustement** par moindres carrés totaux.
4. **Fusion** des morceaux voisins colinéaires.
5. **Filtrage :** au moins 6 points et 0,25 m.

Les paramètres sont ceux de l'onglet Segments, pour que les deux onglets donnent les mêmes segments.

**Correction faite en construisant la carte (partie 4).** Un mur sortait parfois en deux morceaux, dont un petit de 48 cm près d'un coin. La cause était le critère d'écart latéral de `areClose` : il prolongeait aussi la droite du petit morceau bruité, et sur 4 m, une inclinaison de 2° donne 16 cm d'écart. Le critère mesure maintenant la distance des extrémités du **segment le plus court** à la droite du **plus long**, qui est la plus fiable. Un test couvre ce cas.

## Étape 4 : ajout et mise à jour des features

Pour chaque segment extrait du nouveau scan, on cherche les features **proches** (`areClose`) :

| Critère | Défaut |
|---|---|
| Angle | ≤ 5° |
| Écart latéral | ≤ 10 cm |
| Trou longitudinal | ≤ 20 cm |

Les tolérances sont un peu plus larges qu'en partie 4, pour absorber les petites erreurs de superposition entre scans.

- **Aucune feature proche → nouvelle feature.** Confiance 1, 1 observation.
- **Une ou plusieurs features proches → fusion.** C'est le cas le plus courant. La fusion est **pondérée par longueur × confiance** :
  - un mur vu 6 fois n'est presque pas déplacé par une observation bruitée ;
  - un segment récent s'ajuste vite.

  Si le segment observé fait le lien entre plusieurs features (un mur auparavant vu en deux morceaux), elles sont toutes réunies en une seule.
- **Mise à jour des propriétés :** la confiance augmente de 1 (une seule fois par scan, plafonnée à 10), `nObs` augmente, `lastSeen` = scan courant.

**Choix entre « remplacer », « conserver » et « fusionner »** (les trois options du document) : la fusion pondérée les couvre toutes. Si l'ancienne feature est très sûre, le résultat est proche de la *conservation*. Si elle est peu sûre, il est proche du *remplacement*.

## Étape 5 : gestion des absences

Méthode du document : on trace le segment **robot → point d'impact** pour chaque faisceau du scan, et on teste son intersection avec les features existantes (`intersect` de la partie 4).

**Une intersection ne compte que si :**
- elle est sur les deux segments ;
- elle est à plus de **15 cm avant l'impact**. Si le faisceau s'arrête sur la feature, c'est qu'elle est *vue*, pas *traversée*. Cette marge absorbe le bruit du lidar et les petites erreurs de pose ;
- le faisceau n'est pas **rasant** (angle avec la feature ≥ 10°). Un faisceau presque parallèle à un mur peut le « traverser » à cause du bruit.

**Les traversées sont regroupées par intervalles** le long de la feature. Deux traversées voisines (à moins de 30 cm) forment un même trou. Un trou n'est pris en compte que s'il est traversé par **au moins 2 faisceaux**, ce qui rend la méthode robuste à un faisceau aberrant.

**Ajustements, comme dans le document :**

| Situation | Action |
|---|---|
| Trou à un bout | **raccourcir** le segment |
| Trou au milieu (une porte qui s'ouvre) | **scinder** le segment en deux. Les deux morceaux gardent les propriétés de l'original. |
| Segment traversé sur toute sa longueur (aucun morceau ≥ 30 cm ne reste) | **diminuer la confiance** de 1, et **supprimer** à 0 |

Ainsi, un meuble vu une seule fois disparaît dès qu'on voit à travers. Un mur vu 5 fois doit être traversé 5 fois avant d'être supprimé.

**Ordre des opérations :** les absences sont traitées **avant** l'association. Ce qui a disparu est retiré avant d'intégrer ce qui est vu. Sinon, un segment fraîchement fusionné avec une observation pourrait être traversé par les faisceaux du même scan.

## Étape 6 : mise à jour continue et nettoyage

À chaque scan : extraction → absences → association / fusion → nettoyage → coins.

**Nettoyage :**
- **Fusion des features redondantes.** Deux features devenues proches (par exemple après plusieurs fusions) sont réunies, avec la même pondération.
- **Durée de vie** *(propriété ajoutée)*. Une feature qui n'a pas été revue depuis plus de N scans est supprimée. Par défaut N = 0, donc durée de vie infinie : avec des scans pris à des positions différentes, un mur peut légitimement ne pas être revu pendant longtemps. Ce réglage est utile pour un robot qui scanne en continu.
- **Longueur minimale** (30 cm). Les features trop courtes sont supprimées.
- **Filtre d'affichage par confiance** *(propriété exploitée)*. On peut n'afficher que les segments de confiance ≥ N. La carte, elle, garde tout.

**Coins (points d'intérêt).** Un coin est créé pour deux segments :
- dont l'angle entre les droites est ≥ 60° ;
- dont les droites se coupent à moins de 30 cm d'une extrémité de chacun.

Son orientation θ est la **bissectrice dirigée vers l'intérieur** des deux segments. Sa confiance est le minimum des deux confiances. Ces coins serviront à la localisation (partie suivante du module).

---

## Résultats

**Salle simulée (tests automatiques) :**
- **Murs vus depuis deux positions :** fusionnés en un seul segment (pas de doublon), avec 2 observations.
- **Porte qui s'ouvre :** le mur est **scindé** et l'ouverture est retrouvée à moins de 15 cm de sa vraie position (x = 3 à 4 m).
- **Caisse retirée :** sa face, vue une seule fois, est supprimée au scan suivant. Vue 3 fois, elle n'est que diminuée (confiance 3 → 2).
- **Mur hors du champ de vision :** il n'est pas modifié. L'absence de mesure n'est pas une preuve d'absence.
- **Coins :** les 4 coins de la salle sont trouvés à moins de 2 cm, avec un angle d'environ 90°.

**Salle réelle (6 scans Teleplot, après superposition automatique) :**

| Étape (onglet Vectorielle, pas à pas) | Résultat |
|---|---|
| Carte finale | 24 segments, 48,3 m au total |
| Segments de confiance ≥ 2 | 11 segments, 42,5 m (88 % de la longueur) : ce sont les murs |
| Segments de confiance 1 | 13 segments, tous courts (0,31 à 0,81 m) : bruit, coins, bords de porte vus une seule fois |
| Coins | 3 coins de confiance 6 (88–90°), vus par les 6 scans, et 3 coins de confiance 1 |
| Temps de calcul | 3 à 11 ms par scan |

La **confiance permet donc de séparer la structure de la salle du bruit**. C'est ce que la partie 4 ne pouvait faire qu'avec un seuil de longueur.

---

## Difficultés rencontrées

1. **Morceaux de mur non fusionnés** à cause du critère latéral : corrigé (voir l'étape 3).
2. **Faisceaux sans retour.** Si rien n'est touché derrière une porte ouverte, le LD06 ne renvoie pas de point. Il n'y a alors pas de faisceau, donc pas de preuve d'absence : la porte « reste fermée » sur la carte. Un mur de couloir derrière la porte est nécessaire pour la voir s'ouvrir (cas reproduit dans les tests).
3. **Choix des marges de la gestion des absences.**
   - Trop petites : le bruit et les erreurs de pose « traversent » les murs, qui se font grignoter.
   - Trop grandes : les petites ouvertures ne sont pas détectées.

   15 cm et 2 faisceaux minimum sont un compromis pour le LD06 (bruit de 1 à 3 cm) et une superposition précise à quelques cm.
4. **Ordre des opérations** (absences avant association) : voir l'étape 5.

## Limites

- **Qualité dépendante des poses.** Comme pour la grid map, une erreur de superposition supérieure à la marge de 15 cm fait « traverser » des murs et les ronge. Le recalage automatique d'un scan sur la carte vectorielle (scan matching point–segment) n'est pas réalisé.
- **Fusion gloutonne.** Le résultat peut légèrement dépendre de l'ordre d'intégration des scans.
- **Pas de cercles ni d'arcs.** Les objets ronds de plus de 10 cm de diamètre seraient approchés par de petits segments.
- **Confiance simple.** C'est un compteur entier, pas une vraie probabilité. Il ne tient pas compte de la distance de mesure (un mur vu à 7 m est moins précis qu'à 1 m) ni du nombre de points.
- **Coins seulement en « L ».** Les intersections en « T » et les coins arrondis ne sont pas détectés.

## Pistes d'amélioration

- Pondérer chaque observation par la qualité de la mesure (distance, nombre de points, erreur d'ajustement).
- Appliquer à la grid map les mêmes idées : décroissance temporelle des cellules et horodatage.
- Utiliser les coins et les segments de forte confiance pour recaler automatiquement les nouveaux scans (localisation).
