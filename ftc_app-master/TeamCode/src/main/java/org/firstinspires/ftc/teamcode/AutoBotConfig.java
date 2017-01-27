package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcontroller.Util.Global.*;
import static org.firstinspires.ftc.teamcode.VuforiaOp.*;

/**
 * Created by Kerfuffle on 1/26/2017.
 */

@TeleOp(name = "AutoBot Config", group = "Menu")
public class AutoBotConfig extends LinearOpMode {

    public void runOpMode()
    {
        while (opModeIsActive())
        {
            menu.displayMenu();

            switch (menu.getCurrentChoiceOf("Team")) {
                case "RED":
                    teamColor = TeamColor.RED;
                    break;
                case "BLUE":
                    teamColor = TeamColor.BLUE;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Proximity")) {
                case "NEAR":

                    break;
                case "FAR":

                    break;
            }

            switch (menu.getCurrentChoiceOf("Shoot?")) {
                case "YES":

                    break;
                case "NO":

                    break;
            }

            switch (menu.getCurrentChoiceOf("Push de CapBall?")) {
                case "YES":

                    break;
                case "NO":

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

                    break;
                case "NO":

                    break;
            }
        }
    }

}
