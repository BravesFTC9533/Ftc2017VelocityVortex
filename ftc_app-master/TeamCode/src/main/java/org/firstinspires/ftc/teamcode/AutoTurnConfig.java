package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.robotcontroller.Util.Global.*;


/**
 * Created by Kerfuffle on 1/14/2017.
 */

public class AutoTurnConfig extends LinearOpMode {

    public void runOpMode()
    {
        turn.setGamepad(gamepad1);
        turn.setTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            turn.displayMenu();

            dist1Time = turn.getCurrentChoiceOf("Dist1")
        }
    }

}
