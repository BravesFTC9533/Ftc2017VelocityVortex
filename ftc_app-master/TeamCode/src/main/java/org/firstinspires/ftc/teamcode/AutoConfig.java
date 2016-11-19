package org.firstinspires.ftc.teamcode;

/**
 * Created by Kerfuffle on 11/19/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

@TeleOp (name = "(MENU) AutoConfig", group = "Menu")
public class AutoConfig extends MMOpMode_Linear  implements FtcMenu.MenuButtons
{
    private enum Alliance {RED, BLUE};						// our alliance
    private enum Proximity {NEAR, FAR};						// proximity to center vortex

    private enum Shoot {YES, NO};							// should we shoot?
    private enum CapBall {YES, NO};							// should we push capball?
    private enum Park {YES, NO};							// should we park?
    private enum DelayStart {YES, NO};						// delay initial movement


    private HalDashboard dashboard;

    private Alliance alliance = Alliance.RED;
    private Proximity proximity = Proximity.NEAR;
    private Shoot shoot = Shoot.YES;
    private CapBall capBall = CapBall.YES;
    private Park park = Park.YES;
    private DelayStart delayStart = DelayStart.YES;


    public void runOpmode()
    {
        super.runOpMode();

        dashboard = getDashboard();

        dashboard.clearDisplay();
        doMenus();

        while(opModeIsActive())
        {
            switch (alliance)
            {
                case RED:
                    //set red team
                    break;
                case BLUE:
                    //set blue team
                    break;
            }

            switch (proximity)
            {
                case NEAR:
                    Minion9533_AutoBot.initialMoveTime = 0.5;
                    Minion9533_AutoBot.pushBallTime = 1;
                    break;
                case FAR:
                    Minion9533_AutoBot.initialMoveTime = 1.1;
                    Minion9533_AutoBot.pushBallTime = 1.5;
                    break;
            }

            switch (shoot)
            {
                case YES:
                    Minion9533_AutoBot.shoot = true;
                    break;
                case NO:
                    Minion9533_AutoBot.shoot = false;
                    break;
            }

            switch (capBall)
            {
                case YES:
                    Minion9533_AutoBot.capBall = true;
                    break;
                case NO:
                    Minion9533_AutoBot.capBall = false;
                    break;
            }

            switch (park)           //not really used?
            {
                case YES:

                    break;
                case NO:

                    break;
            }

            switch (delayStart)
            {
                case YES:
                    Minion9533_AutoBot.delayStartTime = 15;
                    break;
                case NO:
                    Minion9533_AutoBot.delayStartTime = 0;
                    break;
            }
        }

        waitForStart();
    }


    private void doMenus()
    {

        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance: ", null, this);
        FtcChoiceMenu proximityMenu = new FtcChoiceMenu("Proximity: ", null, this);
        FtcChoiceMenu shootMenu = new FtcChoiceMenu("Shoot?: ", null, this);
        FtcChoiceMenu capBallMenu = new FtcChoiceMenu("Cap de Ball?: ", null, this);
        FtcChoiceMenu parkMenu = new FtcChoiceMenu("Park?: ", null, this);
        FtcChoiceMenu delayStartMenu = new FtcChoiceMenu("DelayStart?", null, this);

        allianceMenu.addChoice("RED", Alliance.RED, null);
        allianceMenu.addChoice("BLUE", Alliance.BLUE, null);

        proximityMenu.addChoice("NEAR", Proximity.NEAR, null);
        proximityMenu.addChoice("FAR", Proximity.FAR, null);

        shootMenu.addChoice("YES", Shoot.YES, null);
        shootMenu.addChoice("NO", Shoot.NO, null);

        capBallMenu.addChoice("YES", CapBall.YES, null);
        capBallMenu.addChoice("NO", CapBall.NO, null);

        parkMenu.addChoice("YES", Park.YES, null);
        parkMenu.addChoice("NO", Park.NO, null);

        delayStartMenu.addChoice("YES", DelayStart.YES, null);
        delayStartMenu.addChoice("NO", DelayStart.NO, null);


        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();
        proximity = (Proximity) proximityMenu.getCurrentChoiceObject();
        shoot = (Shoot) shootMenu.getCurrentChoiceObject();
        capBall = (CapBall) capBallMenu.getCurrentChoiceObject();
        park = (Park) parkMenu.getCurrentChoiceObject();
        delayStart = (DelayStart) delayStartMenu.getCurrentChoiceObject();


        dashboard.displayPrintf(0, "Alliance: %s", allianceMenu.getCurrentChoiceText());
        dashboard.displayPrintf(1, "Proximity: %s", proximityMenu.getCurrentChoiceText());
        dashboard.displayPrintf(2, "Shoot?: %s", shootMenu.getCurrentChoiceText());
        dashboard.displayPrintf(3, "Push de CapBall?: %s", capBallMenu.getCurrentChoiceText());
        dashboard.displayPrintf(4, "Park?: %s", parkMenu.getCurrentChoiceText());
        dashboard.displayPrintf(5, "DelayStart?: %s", delayStartMenu.getCurrentChoiceText());
    }


    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton
}
