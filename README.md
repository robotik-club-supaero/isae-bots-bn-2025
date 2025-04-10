# isae-bots-bn-2025

## Branches :

- `main` : Branche principale
    * Elle n'est pas mise à jour souvent
    * Ne faites pas de commit directement dessus

- `Cocinelle_2025` : Branche de développement des Coccinelles
- `br` : Branche de développement de la BR
- `camera_deportee` : Branche de développement de la caméra déportée (sur le mat)

### Branches à venir :

- Branche de développement des actionneurs 2025

### Branches OBSOLETES

- `br_new` : Ne plus utiliser, sera supprimée bientôt
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