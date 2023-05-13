# FRC 8775 Robotics - 2023 Offseason Code
This repository maintains our robot's 2023 offseason code, designed to meet the requirements of the '[Charged Up](https://www.youtube.com/watch?v=0zpflsYc4PA)' challange.

## Necessary Software
There is some software that you will need to work on the project. It is recommended that you install software as instructed in on WPILIB's [website](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html). This will setup the necessary tools for you to develop in vscode.

If you want however, you can develop in any environment that suites you, as long as you download the necessary components (JDK, etc.).

## Documentation for WPILIB and other libraries
Documentation for of the important libraries used in this project.
 - [WPILIB Documentation](https://docs.wpilib.org/en/stable/index.html) - This is the FRC provided control system for our robot. It is very important to understand how it works both software and hardware. Be sure to check out the [Command-Based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) section as that is the system that is used in this project.

## Branch Protection Rules
Certain branches are protected by rules that must be obeyed

- `main` - All changes can be merged via pull request from created branch into `main` once code has been succesfully tested on robot.

## Running Unit Tests
Unit tests can be run from the command line by using the gradle test task.\
If you are on Windows: `.\gradlew.bat clean test`\
If you are on Linux: `.\gradlew clean test`

## Resolving Merge Conflicts

See something like this?

![image](https://user-images.githubusercontent.com/58612/178773622-c5c66379-4020-47f0-aa52-68d22b86744e.png)

DO NOT click that "Resolve Conflicts" button. Unfortunately GitHub makes resolving merge conflicts harder than it needs to
be, but don't worry! You can follow these steps and get your branch up to date
quickly.

For a full explanation you can watch [this video](https://www.youtube.com/watch?v=I0hUvy7SW6M). She shows an example and explains the whole process really well.

To do this you will need to run some commands in a terminal. VS Code has one you can access or you can use your system's terminal emulator (Windows Terminal on Windows 10+ or Terminal on OS X).

1. Ensure your local main is up to date.

```
git checkout main
git pull
```

2. Switch to your branch. Make sure to use your actual branch name in the command.

```
git checkout your-branch-name
```

3. Replay your commits on top of the current main. This is different from what GitHub instructs you to do, but this is the "correct" way to bring a branch up to date cleanly.

```
git rebase main
```

When you run this command it may ask you to manually resolve merge conflicts. It will show you what the change on main was alongside your change that conflicts with it. You'll need to manually edited those then `git add` each file once it looks good.

4. Update your remote branch with the changes.

```
git push --force-with-lease origin your-branch-name
```

That should do it! If you run into any issues with this please ask in our Discord server and someone will help you out.
