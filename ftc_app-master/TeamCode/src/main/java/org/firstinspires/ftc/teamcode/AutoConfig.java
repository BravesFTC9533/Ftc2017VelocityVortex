package org.firstinspires.ftc.teamcode;

/**
 * Created by Kerfuffle on 11/19/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.FtcSimpleMenu;
import static org.firstinspires.ftc.teamcode.Util.Global.*;

@TeleOp (name = "(MENU) AutoConfig", group = "Menu")
public class AutoConfig extends LinearOpMode {

    public static FtcSimpleMenu menu = new FtcSimpleMenu("Autonomous Configuration");

    @Override
    public void runOpMode() {

        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        menu.addOption("Team", new String[]{"RED", "BLUE"});
        menu.addOption("Proximity", new String[]{"NEAR", "FAR"});
        menu.addOption("Shoot?", new String[]{"YES", "NO"});
        menu.addOption("Push de CapBall?", new String[]{"YES", "NO"});
        menu.addOption("Park?", new String[]{"YES", "NO"});
        menu.addOption("Delay Start?", new String[]{"NO", "YES"});

        /*if (lastMenuConfig != null)
        {
            telemetry.addData("Happenin", "");
            menu.loadFrom(lastMenuConfig);
        }*/


        waitForStart();

        while (opModeIsActive()) {

            menu.displayMenu();

            switch (menu.getCurrentChoiceOf("Team")) {
                case "RED":
                    //set red team
                    break;
                case "BLUE":
                    //set blue team
                    break;
            }

            switch (menu.getCurrentChoiceOf("Proximity")) {
                case "NEAR":
                    Minion9533_AutoBot.initialMoveTime = 0.5;
                    Minion9533_AutoBot.pushBallTime = 1;
                    break;
                case "FAR":
                    Minion9533_AutoBot.initialMoveTime = 1.1;
                    Minion9533_AutoBot.pushBallTime = 1.5;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Shoot?")) {
                case "YES":
                    Minion9533_AutoBot.shoot = true;
                    break;
                case "NO":
                    Minion9533_AutoBot.shoot = false;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Push de CapBall?")) {
                case "YES":
                    Minion9533_AutoBot.capBall = true;
                    break;
                case "NO":
                    Minion9533_AutoBot.capBall = false;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Park"))           //not really used?
            {
                case "YES":

                    break;
                case "NO":

                    break;
            }

            switch (menu.getCurrentChoiceOf("Delay Start?")) {
                case "YES":
                    Minion9533_AutoBot.delayStartTime = 15;
                    break;
                case "NO":
                    Minion9533_AutoBot.delayStartTime = 0;
                    break;
            }
        }


        //lastMenuConfig = menu.getOptionsConfig();
        menu.clearOptions();

    }
}