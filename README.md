# isae-bots-bn-2025

## Branches :

- `main` : Branche principale
    * Elle n'est pas mise à jour souvent
    * Ne faites pas de commit directement dessus
- `PAMI_2026` : Branche de développement des PAMI avec base de code
- `br` : Branche de développement de la BR
- `act` : Branche de développement des actionneurs
- `camera_deportee` : Branche de développement de la caméra déportée (sur le mat)
- `teensy_4.0_test` : Branche de développement du code sur teensy 4.0 (config platformio ect..)

### Branches à venir :

### Branches OBSOLETES

- `main_backup` : Sauvegarde de la branche `main`, ne sera pas mise à jour, peut être supprimée à tout moment

# Fusion des branches de développement avec main

Comme chaque branche de développement est censée modifier un répertoire différent, la fusion devrait se faire sans conflit.

* Vérifier que vous êtes à jour :

```bash
# Sur la branche de développement
git fetch
git rebase origin/main
```

* Aller sur la branche principale :
```
git checkout main
```

* Mettre à jour main :

La branche `br` doit être fusionnée avec une commande spéciale :
```
git subtree pull --prefix=BR origin br --squash
```

Pour les autres branches :
```bash
git merge <dev_branch_name> # ou 'git rebase <dev_branch_name>'
```