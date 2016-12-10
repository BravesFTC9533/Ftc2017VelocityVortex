package org.firstinspires.ftc.teamcode;

/**
 * Created by Kerfuffle on 11/19/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.robotcontroller.Util.Global.*;

@TeleOp (name = "(MENU) AutoConfig", group = "Menu")
public class AutoConfig extends LinearOpMode {


    @Override
    public void runOpMode() {

        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);



        /*if (lastMenuConfig != null)
        {
            telemetry.addData("Happenin", "");
            menu.loadFrom(lastMenuConfig);
        }*/


        waitForStart();

        while (opModeIsActive()) {

            menu.displayMenu();

            //pushBallTime = Double.parseDouble(menu.getCurrentChoiceOf("PushBall Time"));
            //initialMoveTime = Double.parseDouble(menu.getCurrentChoiceOf("Initial Move Time"));

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
                    initialMoveTime = 0.8;
                    pushBallTime = 3;
                    break;
                case "FAR":
                    initialMoveTime = 2;
                    pushBallTime = 2;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Shoot?")) {
                case "YES":
                    shoot = true;
                    break;
                case "NO":
                    shoot = false;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Push de CapBall?")) {
                case "YES":
                    capBall = true;
                    break;
                case "NO":
                    capBall = false;
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
                    delayStartTime = 15;
                    break;
                case "NO":
                    delayStartTime = 0;
                    break;
            }
        }


        //lastMenuConfig = menu.getOptionsConfig();
        //menu.clearOptions();

    }
}