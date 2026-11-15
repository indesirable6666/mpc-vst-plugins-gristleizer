# EMX-1 Virtual : instructions permanentes

## Projet

Émulation avancée de la Korg Electribe EMX-1 (bleue) en plugin **VST2 Linux ARM 32 bits** pour une **Akai MPC Live II** sous **MPC OS 2.15** (firmware à conserver absolument), avec Hakai et accès SSH. Pas de MPC Plugin SDK officiel, pas de MPC OS 3.

Base technique : fork du dépôt `open-mpc/mpc-vst-plugins` (wrapper VST2, outils de build, skins, installateur). Développement dans GitHub Codespaces, installation depuis Windows (application d'installation du dépôt ou zip de release).

Le cahier des charges complet est dans `docs/CAHIER_DES_CHARGES.md` (à y copier). L'audit technique est la première mission ; aucune programmation complète avant sa validation.

## Machine mesurée (15 novembre 2026)

- MPC Live II, hostname `mpc-live-ii`, Linux 5.15.118 temps réel, Buildroot 2021.02.12.
- ARMv7 32 bits (`armv7l`), 4 cœurs en ligne, 1,608 GHz max, NEON et VFPv4.
- RAM : environ 2 Go, environ 1,5 Go disponibles au repos.
- glibc 2.32. Service MPC : `inmusic-mpc` (`/usr/bin/MPC`).
- Racine du système en lecture seule et pleine à 100 %.
- Plugins : `/media/acvs-synths/Synths`, lien vers la carte SD `/media/az01-internal-sd` (ext4).
- Mémoire interne : `/media/az01-internal` (réglages MPC, mises à jour, système).
- Autres disques sous `/media` : exFAT, noms avec espaces et caractères spéciaux.
- Samples de l'utilisateur : dans `MPC Documents/Projects/<projet>_[ProjectData]` sur la SD (extension `.WAV` en majuscules). Le dossier `Samples` est vide.

## Règles absolues

1. Ne jamais écrire à la racine du système ni dans `/media/az01-internal`. Seule exception : la ligne ajoutée à `MPC.settings` par l'installateur, avec sauvegarde.
2. Toutes les données du plugin (presets, patterns, réglages) vont dans son propre dossier sur la SD, à côté du plugin.
3. Ne jamais modifier le firmware, les partitions ni les fichiers de configuration Akai sans nécessité démontrée.
4. Ne jamais exécuter de commande SSH destructive sans validation explicite de l'utilisateur.
5. Ne jamais supprimer ni remplacer un plugin existant.
6. Sauvegarde avant chaque installation. Prévoir une procédure de désinstallation pour chaque version.
7. Ne jamais modifier les samples originaux de l'utilisateur (lecture seule).
8. Ne jamais inventer une API ni supposer une compatibilité non vérifiée. Lire la documentation avant d'utiliser une bibliothèque ou un outil.
9. Un plantage du plugin arrête toute la MPC : sauvegarder le projet MPC avant chaque test.
10. Aucune ROM Korg ne doit être extraite ni distribuée. Les sons de référence viennent d'enregistrements de l'utilisateur sur sa propre machine, pour un usage personnel.

## Règles audio

- Aucune allocation mémoire, aucun verrou bloquant et aucune lecture de disque dans le fil audio.
- Chargement des samples dans un thread séparé, avec un plafond de mémoire (laisser de la RAM à Akai).
- Le fil audio dispose d'environ 2,9 ms par bloc de 128 images à 44,1 kHz, partagées avec le reste de la MPC. La stabilité passe avant le nombre de voix.
- Polyphonie réglable (4, 8, 16). Les 40 voix sont un objectif à confirmer par benchmark, jamais une obligation.

## Décisions de l'utilisateur

- Interface fidèle à l'EMX-1 bleue, sans être lourde : face avant, 16 boutons de pas, chenillard de lecture, textes en direct. Les mises à jour de l'écran ne se font qu'aux changements, jamais en animation continue.
- Le chenillard et l'affichage sont à tester très tôt sur la MPC ; en cas d'échec, présenter un compromis à l'utilisateur avant de continuer.
- Samples : le preset garde le chemin, la taille et la durée. Message clair si le fichier manque. Les chemins sont découverts dynamiquement sous `/media` (espaces et caractères spéciaux possibles, extensions insensibles à la casse).

## À lire avant d'écrire du code

`CLAUDE.md` du dépôt et `.claude/skills/mpc-vst-plugin`, `docs/NOTES.md` (source de vérité), `docs/OS2_SKINS.md`, `docs/PORTING.md`, `docs/BENCH.md`, `docs/MIDI_TIMING.md`, `wrapper/engine.h`, `wrapper/vst2_wrap.c`, `poc/synth.c`.

## À vérifier (non confirmé)

- Le moteur peut-il être écrit en C++ derrière l'interface du wrapper ?
- Le rendu est-il limité à des entiers 16 bits ? Impact sur le Valve Force et les effets.
- Format de skin 2.x accepté sur cette MPC (commande `tools/skin_compat.py`).
- Où l'installateur range la sauvegarde de `MPC.settings`.
- Nombre de voix tenable (`tools/bench.sh`).
- Chenillard : un contrôle affiché ou masqué selon un paramètre suffit-il, à quel coût ?

## Méthode

Travailler par phases : plugin minimal, synthé polyphonique, User Sample, drums, filtres et modulations, effets, séquenceur, interface finale, optimisation. Compiler après chaque modification importante, tester sur la MPC dès que possible, lancer `bench.sh` à chaque palier.

Utiliser Git. Tenir à jour `CHANGELOG.md`, `TODO.md` et `TECHNICAL_NOTES.md`. Documenter chaque version.

## Façon de travailler avec l'utilisateur

Répondre en français. L'utilisateur est débutant : expliquer chaque étape, une à la fois, avec la commande exacte et le résultat attendu. Attendre sa validation avant de passer à la suite.
