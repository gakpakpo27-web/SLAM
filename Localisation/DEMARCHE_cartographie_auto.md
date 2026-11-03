# Cartographie automatique : refaire la carte au sol, vite

**Outil :** onglet **Scans**, section « Cartographie automatique ».
**Code :** `createMapper` et `mapperUpdate` dans `lidar_loc.js`.
**Banc :** `node bench_cartographie.js`, résultats dans `resultats_cartographie.md`.
**Test :** 1 test dans `test_lidar_core.js` (117 au total).

## 1. Pourquoi refaire la carte

Au sol, le robot se perdait. La raison : la carte et les scans ne décrivaient pas la même salle.
- **Un lidar 2D ne voit qu'une tranche horizontale.** Une carte faite robot soulevé contient les plateaux de tables, les murs au-dessus des chaises… Au sol, le lidar voit des pieds.
- **Les méthodes essayées ne compensent pas cet écart** (filtrer les pieds, pénaliser les faisceaux qui traversent un mur, préférer la position prévue). Les chiffres sont dans `DEMARCHE_jour2_localisation.md` §7.5.

La solution la plus efficace est de **refaire la carte dans les conditions d'utilisation** : robot au sol, avec tous les obstacles présents. Pour aller vite, la cartographie est automatique.

## 2. Méthode (SLAM simple par images clés)

1. **Flux continu.** Le lidar tourne et chaque scan reçu est traité, en USB comme en WiFi.
2. **Recalage sur la carte déjà construite.** On part de la position prévue (la position précédente, plus la commande de pilotage WiFi si elle est connue), puis :
   - recherche multi-résolution dans une fenêtre de ±50 cm et ±25° ;
   - affinage par descente de gradient, qui ne doit pas s'écarter de plus de 15 cm ou 5° du résultat de la recherche (sinon une vue tournée de 90° peut l'emporter dans un coin encombré) ;
   - préférence pour la solution proche de la position prévue : une solution lointaine n'est retenue que si elle colle nettement mieux (10 points de pourcentage).

   Recaler sur la **carte** plutôt que sur le scan précédent limite la dérive.
3. **Qualité.** On mesure la part des points qui collent, **parmi ceux qui tombent dans une zone déjà vue** de la carte. Les points qui découvrent une zone nouvelle ne comptent ni pour ni contre. Sans cette règle, une position restée en arrière paraît meilleure, et le robot semble ne plus avancer.
4. **Images clés.** On enregistre une nouvelle position dès que le robot a avancé de 40 cm ou tourné de 25° depuis la précédente. Tant qu'il reste immobile (à 3 cm et 1,5° près), les tours suivants y sont **accumulés**, jusqu'à 3 par défaut, pour avoir plus de points. Chaque position devient un scan de la session (nommé `autoN_01`, `autoN_02`…, verrouillé), et elle est intégrée à la grid map, agrandie si besoin.
5. **Perdu.** Si moins de 25 % des points collent, aucune position n'est enregistrée, et on suit le déplacement prévu. Une relocalisation globale n'est tentée que :
   - sur une carte d'au moins 5 positions ;
   - et près de la position prévue (moins de 1 m), sauf si le robot est perdu depuis longtemps.

   Sans ces gardes, une relocalisation sur la carte d'une seule position plaçait le robot à 1,2 m de la vérité, et toute la carte partait décalée.

## 3. Résultats (simulation, robot au sol)

Salle vue au sol : pieds de 14 tables, 60 chaises, des sacs, des personnes. Trajet de 440 scans, odométrie bruitée de 10 %.

| Graine | Positions enregistrées | Temps | Erreur des positions (médiane / max) | Localisation des scans du sol : carte faite soulevé | … carte automatique au sol |
|---|---|---|---|---|---|
| 3 | 87 | 13 s | 3,5 cm / 14 cm | 14/15 | **15/15** |
| 4 | 86 | 13 s | 1,8 cm / 4,0 cm | 15/15 | **15/15** |
| 5 | 87 | 13 s | 2,0 cm / 3,5 cm | 15/15 | **15/15** |

Dans la page, avec le robot simulé, la carte se construit toute seule : 87 positions et des murs propres.

## 4. Limites et conseils

- **Démarrer dans un endroit dégagé, robot immobile 2 secondes.** Au tout début, la carte ne contient qu'un point de vue. Entouré de pieds serrés, les pieds vus (ou ratés) changent dès que le robot bouge, et le recalage peut se tromper. Ce cas a été reproduit en simulation : la vraie position y collait moins bien qu'une position fausse.
- **Rouler doucement**, et repasser par des zones déjà cartographiées. Il n'y a pas encore de fermeture de boucle : sur un très long trajet, une petite dérive peut s'accumuler.
- **Repère.** Le repère de la nouvelle carte est celui du premier scan : la carte peut être un peu tournée. L'onglet Traitement (plan, murs nets) se cale de toute façon sur les axes de la salle.
- **Vitesses de la prédiction** (onglet Robot en direct : 0,25 m/s, 90°/s) : elles sont à mesurer sur ton robot. Une erreur n'est pas grave, le recalage tolère ±50 cm et ±25°.
- **Ensuite :** *Construire la grid map*, puis l'onglet **Traitement** pour nettoyer et obtenir le plan, et l'onglet **Robot en direct** pour se localiser sur cette nouvelle carte, faite à la bonne hauteur.
