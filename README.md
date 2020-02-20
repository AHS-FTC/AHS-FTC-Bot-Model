# AHS-FTC-Bot-Model
The official repo and BotModel of FTC Team 16896 Black Forest Robotics

AHS-FTC-Bot-Model is a class structure and testing interface for First Tech Challenge. Or it used to be. It's just some jumbled code now.
Meshed in with the model is BFR's Skystone code, opmodes included. 

You can find us on Instagram @black_forest_robotics, on [YouTube](https://www.youtube.com/channel/UCpi8xaNKtnHhdTxuc3xE_hA), and on [The Orange Alliance](https://theorangealliance.org/teams/16896). Questions? Send us an email at blackforestrobotics@gmail.com.

## Install Instructions:

Download the FTC SDK from: https://github.com/FIRST-Tech-Challenge/SkyStone (alternatively you can use git for this)

Unpack the FTC SDK zip file into a known location. You probably want this location to be clean and accessible.

Open up Android Studio (File > Close Project if you have a project open already) and select 'Import project (Gradle, Eclipse ADT, etc.)'

Navigate to the file where you unpacked your FTC SDK. Select the Skystone-Master project, which should be signified by the Android Studio logo.

Open up the Android Studio terminal, which should be located near the bottom, slight left of your screen.

Check your file path. It needs to be at the top level of your project, the SkyStone-Master folder. If it isn't use the 'cd..' command to navigate upwards.

Go to said SkyStone-Master folder in Android Studio or your preffered file viewer and delete the README.md file at this level. Also delete the README.md in the teamcode package. Burn all the readmes!

In the Android Studio terminal use the following commands:
```
git init
git remote add origin https://github.com/AHS-FTC/AHS-FTC-Bot-Model.git
git fetch --all --prune
git checkout <branch name>
git pull origin <branch name> 
```
Make the branch name whatever branch you're pulling from in the origin. Once you do this, you should have the packages and code from this repo.

##### Add the JUnit dependency

Now navigate to the build.common.gradle file in your android studio gradle scripts likely on the left side of the screen.

In the last line of this file, add the following code:

```
dependencies{
   testImplementation 'junit:junit:4.12'
}
```

Sync your gradle. (A blue bar should pop up giving you the option. If not, then it's the elephant on the top right.)

And you're done! Build your project and run some tests to make sure everything worked. If something broke, start fresh by re-unpacking the SDK and try everything again.
