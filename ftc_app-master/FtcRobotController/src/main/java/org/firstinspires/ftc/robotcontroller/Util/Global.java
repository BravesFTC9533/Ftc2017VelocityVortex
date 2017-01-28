package org.firstinspires.ftc.robotcontroller.Util;

import android.content.Context;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by Kerfuffle on 12/9/2016.
 */

public class Global {

    //public static final int RED = 0, BLUE = 1;

    public static FtcSimpleMenu menu = new FtcSimpleMenu("Autonomous Menu");
    //public static FtcSimpleMenu turn = new FtcSimpleMenu("Autonomous Turn");

    public static FtcSimpleMenu tests = new FtcSimpleMenu("Tests");

    public static float compass;
    //public static boolean opModeActive = false;

    public static Context context;


    /*public static double initialMoveTime = 0.8;  //defaults to near
    public static double pushBallTime = 3;      //defaults to near
    public static boolean shoot = true;
    public static boolean capBall = true;
    public static boolean park = true;
    public static double delayStartTime = 0;
    public static int team = RED;


    public static double dist1Time = 0;
    public static double dist2Time = 0;
    public static double turnDeg = 0;*/


    public static void init()
    {
        menu.addOption("Team", new String[]{"RED", "BLUE"});
        menu.addOption("Proximity", new String[]{"NEAR", "FAR"});
        menu.addOption("Beacons", 2, 0, 1, 2);
//        menu.addOption("Initial Move Time", 30.0, 0.0, 0.1);
//        menu.addOption("PushBall Time", 30.0, 0.0, 0.1);
//        menu.addOption("Shoot", new String[]{"YES", "NO"});
//        menu.addOption("Push CapBall", new String[]{"YES", "NO"});
//        menu.addOption("Park", new String[]{"YES", "NO"});
//        menu.addOption("Delay Start", 30, 0, 0.1);


        tests.addOption("Shoot Power", 1, 0, 0.1, 1);
        tests.addOption("Elevator Down Power", 1, 0, 0.05, 0.1);

        /*turn.addOption("Dist1", 10, 0, 0.5);
        turn.addOption("Degrees", 360, 0, 5);
        turn.addOption("Dist2", 10, 0, 0.5);*/
    }

}
