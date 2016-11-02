# ECE 4760 Final Project

## A Quick Git Guide:

### Cloning this repository:

1. If you do not have git installed on your computer, install git from the web.
2. Open Terminal/Console on your computer.
3. Navigate to the directory to which you want the repo to be cloned.
 1. Typing `cd dirname` in the console, where `dirname` is the name of a directory, will change
your current working directory to that directory.
 2. Typing `ls` will show you the contents of the current working directory, which can be helpful
for discovering which directoreis you can `cd` to.
 3. Typing `pwd` will print the current working directory. This command will help you discover
what directory you are starting in/if you have navigated to the right place.
 4. Typing `cd ..` will change your directory up one level. If you are in `Desktop/projects`, 
typing `cd ..` will change your cwd to `Desktop`.
4. Type this command: `git clone https://github.com/enjmusic/ece4760-final.git`.
5. Once the printout stops for the `git clone` command, you should have a copy of the repo on
your local machine. `cd` to it using `cd ece4760-final`.

### Pushing/pulling changes:

1. Before you start working, always remember to `git pull` from Terminal/Console while in
the repo directory.
2. Before you push changes, always remember to `git pull` as mentioned above.
3. If two people are working on the same file and both have different unpushed changes,
this can cause merge conflicts. Try not to change the same files at the same time. If you
do, contact me (Ian) and I will do my best to help resolve the merge conflict.
4. When you are ready to commit changes to the remote repository (online), first type `git status`.
This will show you the current version-control status of your local repository. If any files are
listed as `untracked`, you should type `git add <filename>` (without the <>). 
5. Once you have added any untracked files that you wish to commit, type `git commit -am "commit message goes here"`, with a message indicating what you have changed in the quotes.
6. Lastly, typing `git push origin master` to push your changes to the origin.

### I hope this helps!
