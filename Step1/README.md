# Step 1: Setting Up Your Environment

Hello! If you are reading this, you have found the `purdue-ece569-spring2026` organization, and you are ready to start work on Lab 1. Please follow along with the rest of the tutorial by reading the instructions carefully. If you have questions, you can use Piazza, or talk to your TA (Logan) during office hours. Please do not contact the TA by email for help with the homework or the labs: that is exactly what Piazza is for.

### Opening the Terminal

On your Ubuntu machine (eceprog or your own), right-click on the desktop background and press `Open Terminal Here`. Now run:
```bash
mkdir -p ~/ece569-spring2026
```
*Note: this course will require the use of the terminal. If you are unfamiliar with a command or any of its flags (such as `-p`), you can add a `--help` flag to the end of the command, or run `man <your command>` to view the manual's entry for that command, if it exists. For example, you can run `man mkdir` to find out more about the `mkdir` command. There are hundreds of guides to using the terminal if you have never used it before. I recommend reading through [this article from ubuntu](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview) to get a really solid foundation.*

### Setup Github

You will need to setup an SSH key to access read/write from Github. Try following the instructions below first. Additional instructions are provided [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) and [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account#adding-a-new-ssh-key-to-your-account) if you get stuck. 

Open a new terminal and run the following (replacing `your_email` with the email associated with your github account):
```bash
ssh-keygen -t ed25519 -C "your_email@purdue.edu"
```
When prompted with "Enter a file in which to save the key", just press `Enter` to accept the default file location. Forthe passphrase, just press `Enter` twice for an empty passphrase.

1. Open the file you created in VSCode
    ```bash
    code ~/.ssh/id_ed25519.pub
    ```
    and then copy the SSH public key to your clipboard. (You can do this with `Ctrl+A`, `Ctrl+C`.)
2. Go to any page on GitHub, click on your profile photo, and click `Settings`. 
3. Under `Access` section of the sidebar, click `SSH and GPG keys`. 
4. Click `New SSH key`
5. In the `Title` field, enter something descriptive such as `eceprog`.
6. Paste your publickey into the "Key" field.
7. Click `Add SSH Key`


Now, you need to set your github name and email. From any terminal, run:
```
git config --global user.name "Your First and Last Name"
git config --global user.email "your_email@purdue.edu"
```
_Be sure to replace your name and email with your actual name and email._

### Duplicate the Lab 1 Repository

In order to complete Lab 1, you will need to duplicate this repository. Go to [this link](https://github.com/purdue-ece569-spring2026/Lab1/) and then click the green `Use the template` button in the top right corner. On the next page, be sure to set the `Owner` to your username, and the `Repository Name` to `Lab1`. See the screenshot below for details. Finally, press `Create repository`. Your own repository should be created in a few seconds.

<img src="lab1_create_repository.png" alt="screenshot of creating repository" width="60%"/>

Once your repository has been created, go to its `Settings -> Collaborators` and click the green `Add people` button and then add `logdog` (the github account name of your TA, Logan Dihel). 

<img src="lab1_give_logan_access.png" alt="screenshot of settings" width="100%"/>

### Clone Your Repository

In your browser, return to your Lab1 repository's main page. Click the green `Code` button, select the `SSH` option, and then click the copy button (two overlapping gray squares). In eceprog, start a new terminal window and run:
```bash
cd ~/ece569-spring2026
git clone <paste your clipboard here with Ctrl+Shift+V>
```
_Note: as a general rule with terminal instructions, you never include the angled brackets `< >` in your commands unless specified otherwise_. Your repository should now be inside the `ece569-spring2026` folder, and you should have read/write access.

### Testing Write Access

Open the repository in VSCode by running:
```bash
code Lab1
```
Then, update the README.md file inside the `ece569-spring2026/Lab1` folder to include your name and email:
```
Name: `your name here`
Email: `your email here`
```
You can now commit and push these changes. If you've used git before and know what you are doing, feel free to use the command line to do this. Otherwise, you can use the `Source Control` built into VSCode to do this easily.

#### Commit and Push using VSCode Source Control

In the left window, click the `Source Control` button (looks like three dots connected by lines; it may a small blue circle near it) or press `Ctrl+Shift+G`.

<img src="lab1_source_control.png" alt="screenshot of settings" width="50%"/>

_Note: if you don't see anything in source control, try closing and then re-openning VSCode a few times to refresh. If this still doesn't work, navigate to your newly downloaded `Lab1` folder, and then run `git remote -v` and verify that the URL is something like `git@github.com:<your_username>/Lab1.git`. Reach out on Piazza for further help if you get stuck._

Add the changes by putting your mouse cursor over `Changes` and pressing the `+` button. Then type a message in the textbox above, and press `Commit`. Then in the botton left corner of VSCode, you can press the `1↑` button to sync your changes with the remote repository on github. 

Visit the your Lab1 repository in a web browser, and hit the refresh button. Verify that the `README.md` file now contains your name and email.

### VSCode Extensions

All of our code for `Lab1` will live in the `ece569-spring2026/Lab1`, which should be open in VSCode now. You can open a new terminal within VSCode by pressing `Ctrl+Shift+~`. I also recommend (but don't require) installing some useful extensions:
* CMake (twxs)
* CMake Tools (Microsoft)
* XML (Red Hat)
* Python

The extensions menu can be brought up by pressing `Ctrl+Shift+X` or simply clicking on the `Extensions` button (looks like 4 squares) on the far left side of your VSCode window.

I also (strongly) recommend you enable autosave (click `File > Auto Save`) from the drop down menu on the top left corner of your VSCode window. This will save you a lot of frustration down the road.

## ROS Setup

This course will require the use of ROS (Robot Operating System), which is an open-source collection of software packages and tools designed for building, testing, and simulating robots. You will learn a lot about ROS in this course. For now, do the following:
```bash
code ~/.bashrc
```
This will open the `.bashrc` file located in the your home directory. This is a special executable file which is automatically run when a new terminal window is opened by you. Paste the following line of code at the bottom of your `.bashrc` file:
```bash
source /opt/ros/humble/setup.bash
```
or if you are using ROS2 Jazzy instead, use
```bash
source /opt/ros/jazzy/setup.bash
```
This will allow each new terminal window you open to recognize ROS commands, and save you the hassle of typing this command in *every single terminal window you open*. Close your `.bashrc` file. Close your current terminal with `Ctrl+D`, and open a new one with `Ctrl+Shift+~` (VSCode shortcuts).

### Specific instructions for eceprog users

If you are using `eceprog` for your ROS development, you will need to set the `ROS_DOMAIN_ID` variable to the unique ID that your TA emailed you. If you don't have a `ROS_DOMAIN_ID` yet then email your TA (Logan) at `ldihel@purdue.edu` asking for one.

Once you have your unique ID, open your `.bashrc` file
```bash
code ~/.bashrc
```
and then beneath the `source /opt/ros/humble/setup.bash` line, set the `ROS_DOMAIN_ID` variable to the unique ID that you were emailed. For example, if your `ROS_DOMAIN_ID` was 9 (yours will be higher), you would write:
```bash
export ROS_DOMAIN_ID=9
```

Close your terminal now, and open a new one. The script will run and output your `ROS_DOMAIN_ID`. Open a new terminal window and verify that 
```bash
echo $ROS_DOMAIN_ID
```
prints your domain ID (an integer between 10 and 101, inclusive - it should not be set to 9).

#### Why ROS_DOMAIN_ID?

In short, each student on eceprog needs to have a unique `ROS_DOMAIN_ID` in order to prevent one student's ROS instance from talking with another. The ROS domain IDs 0-9 have been reserved for other students (not in ECE 569) on eceprog, while domain IDs 10-101 have been reserved for ECE 569 students.

## Next Steps

Proceed to [Step 2](/Step2/README.md)
