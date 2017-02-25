package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.Helpers;

/**
 * Created by Kerfuffle on 2/12/2017.
 */

@Autonomous(name = "ODS", group = "Test")
public class TestODS extends MMOpMode_Linear {

    private double TARGET_LIGHT = 0.4;

    private enum Direction
    {
        LEFT, RIGHT;
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();

        Direction lastDir = null;

        waitForStart();

        while (opModeIsActive())
        {

            if (robot.ods.getLightDetected() > TARGET_LIGHT)
            {
                // on line
            }
            else
            {
                // not on line

                if (lastDir == Direction.LEFT)
                {
                    // go right
                }
                else if (lastDir == Direction.RIGHT)
                {
                    // go left
                }
            }

            robot.dashboard.displayText(1,"" + robot.ods.getLightDetected());
        }
    }

}
