# Servoteknikk
Servoteknikk 2025

Servoteknikk Repository Setup Guide
====================================

Welcome to the Servoteknikk project!
This guide explains how to set up Git, configure SSH, and connect to this GitHub repository so everyone can push and pull code without typing their username or password each time.

------------------------------------------------------------

Step 1: Check for an existing SSH key
------------------------------------
Open Git Bash and run:
    ls -al ~/.ssh

If you see files like id_ed25519 and id_ed25519.pub, you already have SSH keys — skip to Step 4.
If not, continue below.

------------------------------------------------------------

Step 2: Generate a new SSH key
------------------------------
Run this command (replace the email with your GitHub email):
    ssh-keygen -t ed25519 -C "your_email@example.com"

Press Enter through all prompts to use the default settings.
Your new SSH key pair will be created at:
    ~/.ssh/id_ed25519

------------------------------------------------------------

Step 3: Add your SSH key to the SSH agent
-----------------------------------------
Start the SSH agent:
    eval "$(ssh-agent -s)"

Then add your key:
    ssh-add ~/.ssh/id_ed25519

------------------------------------------------------------

Step 4: Add your public key to GitHub
------------------------------------
1. Display your public key:
       cat ~/.ssh/id_ed25519.pub
2. Copy everything printed (it starts with ssh-ed25519).
3. Go to GitHub:
   - Click your profile picture → Settings → SSH and GPG keys → New SSH key
4. Paste your key, give it a meaningful title (for example, "Laptop - Windows"), and choose Authentication Key.
5. Click Add SSH key.

------------------------------------------------------------

Step 5: Test your SSH connection
-------------------------------
Run this command:
    ssh -T git@github.com

If prompted, type "yes" and press Enter.
You should see a message like:
    Hi <your-username>! You've successfully authenticated, but GitHub does not provide shell access.

That means your SSH setup is working correctly.

------------------------------------------------------------

Step 6: Clone the repository
----------------------------
Decide where you want to store the project (for example, Desktop):
    cd ~/Desktop

Clone the repository:
    git clone git@github.com:awolokhoff-art/Servoteknikk.git

Move into the folder:
    cd Servoteknikk

------------------------------------------------------------

Step 7: Verify that it’s connected
----------------------------------
Check that your remote repository is linked correctly:
    git remote -v

You should see something like:
    origin  git@github.com:awolokhoff-art/Servoteknikk.git (fetch)
    origin  git@github.com:awolokhoff-art/Servoteknikk.git (push)

------------------------------------------------------------

Step 8: Basic Git workflow
--------------------------
Whenever you make changes:
    git add .
    git commit -m "Describe your changes"
    git push origin main

To update your local version with the latest from others:
    git pull

------------------------------------------------------------

Done!
-----
You’re all set!
Everyone on the team can now securely clone, push, and pull using SSH.

If you encounter issues, recheck:
- The SSH key is added to your GitHub account.
- The key is added to your SSH agent (ssh-add -l).
- You’re using the SSH URL (git@github.com:...) and not the HTTPS one.

Happy coding!
