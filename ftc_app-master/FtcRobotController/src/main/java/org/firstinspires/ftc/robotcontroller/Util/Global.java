package org.firstinspires.ftc.robotcontroller.Util;

import android.content.Context;

/**
 * Created by Kerfuffle on 12/9/2016.
 */

public class Global {

    public static FtcSimpleMenu menu = new FtcSimpleMenu("Autonomous Menu");

    public static Context context;

    public static void init()
    {
        menu.addOption("Team", new String[]{"RED", "BLUE"});
        menu.addOption("Proximity", new String[]{"NEAR", "FAR"});
        menu.addOption("Shoot?", new String[]{"YES", "NO"});
        menu.addOption("Push de CapBall?", new String[]{"YES", "NO"});
        menu.addOption("Park?", new String[]{"YES", "NO"});
        menu.addOption("Delay Start?", new String[]{"NO", "YES"});
    }

}
