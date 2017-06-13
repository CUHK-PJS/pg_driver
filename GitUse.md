# Basic Useage for github

## Github basic usage
* clone
```bash
git clone <your-repo-url>.git
```
If you couldn't clone the repo, need to add the ssh id_pub key into the github ssh account.

### update the current local repo from the remote
```bash
git pull origin <branch>
```
Here, the <branch> is your current branch, default is master

### add modification to the current local repo
```bash
git add -A
git commit -m "your commits"
```
### push current local modifications to the remote
```bash
git push origin <branch>
```
### build new branch
Usually we will have several branches under a same repo
```bash
git checkout -b <branch>
```
You can also check the branches you have
```bash
git branch
```
To switch to another repo
```bash
git checkout <branch>
```
