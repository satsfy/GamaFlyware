# GamaFlyware container codebase

## Configuring Display

Set the right display with:
```bash
export DISPLAY=:0 # in ubuntu, check what `echo $DISPLAY` spits out outside the repo and paste here 
```

## Configuring the deps

You need the following repos in `px4_msgs_ws` folder (provided by the docker container already).

- px4_msgs in branch release/1.15
- px4_ros_com in branch main

These need to be compiled with `colcon` so `ws` can use it for communication with PX4-Autopilot.

## Updating upstream

The `gz` folder is the remote repo https://github.com/PX4/PX4-gazebo-models/ with modifications I included. To update to upstream master, run:

```
git fetch gz-upstream
git subtree pull --prefix=gz gz-upstream main --squash
``` 

## Running QGroundControl

```bash
su qgcuser;
# now as user 'qgcuser'
./QGroundControl.AppImage
```

In case of failures, run `xhost +local:` in your host machine.
Then run:
```bash
su - qgcuser
export DISPLAY=:1 # here you can also try putting ':0' instead
cd /sitl
./QGroundControl.AppImage
```

This command should open QGroundControl from inside the container.

Results may vary in other operating systems.

## `git subtree` - adding an upstream tracked repo with git support for local modifications

* `gz/` tracked as a **normal folder** in the main repo
* no separate checkout for collaborators
* ability to **pull upstream updates into just `gz/`**
* ability to keep your own local files/changes inside `gz/`

What you **cannot** do is keep `gz/.git` there and also have the parent repo track `gz/` as an ordinary directory. With that `.git` directory present, `gz` is a nested repo, not normal contents.

Back it up first.

```bash
mv gz gz_backup
```

Add the upstream remote to the parent repo. From the **main repo root**:

```bash
git remote add gz-upstream <URL-OF-GZ-UPSTREAM>
git fetch gz-upstream
```

If the remote already exists, skip the add.

Import `gz/` as a subtree. This creates `gz/` as a normal tracked folder in the parent repo:

```bash
git subtree add --prefix=gz gz-upstream <branch> --squash
```

Now compare your old `gz_backup/` with the new subtree-imported `gz/`. Copy changes back (caution):

```bash
rsync -a --exclude=.git gz_backup/ gz/
```

Then inspect carefully. Commit those local customizations as a separate commit. Then remove the backup when satisfied:

```bash
rm -rf gz_backup
```

**To update upstream changes**: 
```bash
git fetch gz-upstream
git subtree pull --prefix=gz gz-upstream <branch> --squash
```

You will be able to resolve conflicts like a normal merge.
