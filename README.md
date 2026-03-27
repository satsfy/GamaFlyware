# GamaFlyware container codebase

## Updating upstream

The `gz` folder is the remote repo https://github.com/PX4/PX4-gazebo-models/ with modifications I included. To update to upstream master, run:

```
git fetch gz-upstream
git subtree pull --prefix=gz gz-upstream main --squash
``` 