package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcontroller.Util.Global.*;
import static org.firstinspires.ftc.teamcode.VuforiaOp.*;

/**
 * Created by Kerfuffle on 1/26/2017.
 */

@TeleOp(name = "(MENU) AutoBot Config", group = "Menu")
public class AutoBotConfig extends LinearOpMode {

    public void runOpMode()
    {
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            menu.displayMenu();

            //numBeacons = (int) (Double.parseDouble(menu.getCurrentChoiceOf("Beacons")));

            switch (menu.getCurrentChoiceOf("Team")) {
                case "RED":
                    teamColor = TeamColor.RED;
                    break;
                case "BLUE":
                    teamColor = TeamColor.BLUE;
                    break;
            }


//            switch (menu.getCurrentChoiceOf("Proximity")) {
//                case "NEAR":
//                    proximity = Proximity.NEAR;
//                    break;
//                case "FAR":
//                    proximity = Proximity.FAR;
//                    break;
//            }
//            switch (menu.getCurrentChoiceOf("Shoot")) {
//                case "YES":
//
//                    break;
//                case "NO":
//
//                    break;
//            }
//
//            switch (menu.getCurrentChoiceOf("Push CapBall")) {
//                case "YES":
//
//                    break;
//                case "NO":
//
//                    break;
//            }
//
//            switch (menu.getCurrentChoiceOf("Park"))           //not really used?
//            {
//                case "YES":
//
//                    break;
//                case "NO":
//
//                    break;
//            }
        }
    }

}
