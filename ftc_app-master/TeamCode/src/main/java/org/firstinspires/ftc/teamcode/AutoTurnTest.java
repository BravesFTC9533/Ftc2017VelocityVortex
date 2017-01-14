package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.Util.Global;
import org.firstinspires.ftc.teamcode.MMOpMode_Linear;
import org.firstinspires.ftc.teamcode.Util.MMCompass;
import org.firstinspires.ftc.teamcode.Util.MMPid;

import hallib.HalDashboard;

/**
 * Created by Kerfuffle on 1/14/2017.
 */

@Autonomous(name="TurnBot", group="Autobot")
public class AutoTurnTest extends MMOpMode_Linear {

    private ElapsedTime runtime = new ElapsedTime();

    private HalDashboard dashboard;

    private MMCompass mmCompass = new MMCompass();

    private MMPid rpid = new MMPid();

    private void goStraight(String step, double time){
        //robot.DriveRobot(FORWARD_SPEED, FORWARD_SPEED, telemetry);

        dashboard.displayPrintf(0, "%s: Drive straight for %s second(s)", step, time);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
            dashboard.displayPrintf(1, "Straight: %2.5f S Elapsed: %s", runtime.seconds(), robot.leftMotor.getCurrentPosition());

            //robot.DriveRobot(FORWARD_SPEED, FORWARD_SPEED);
            mechDrive.Drive(0.4, 0, 0);
            robot.waitForTick(40);
        }

        mechDrive.Stop();
        pauseBetweenSteps();
    }

    private void pauseBetweenSteps(){
        waitFor(1);
    }

    private void waitFor(double seconds){
        robot.waitForTick((long)(seconds * 1000));
    }

    private void turn(double angle)
    {
        double compassAngle = Global.compass;
        double turnAngle = angle + compassAngle;

        //watch out for going over 360 deg

        if (angle > 0)      // going counter clockwise
        {
            while (compassAngle < turnAngle)
            {
                compassAngle = Global.compass;

                mechDrive.Drive(0,0,0.4, false);
            }
        }
        else if (angle < 0)     // going clockwise
        {
            while (compassAngle > turnAngle)
            {
                compassAngle = Global.compass;

                mechDrive.Drive(0,0,0.4, false);
            }
        }


        mechDrive.Stop();
        pauseBetweenSteps();
    }

    public void runOpMode()
    {
        super.runOpMode();

        rpid.SetTunings(0.8, 0.2, 0.1);

        dashboard = getDashboard();


        rpid.SetOutputLimits(-1, 1);


        waitForStart();


        int step = 0;

        while (opModeIsActive())
        {


            switch (step) {
                case 0:
                    goStraight("1st Move", Global.dist1Time);
                    step = 1;
                    break;
                case 1:
                    TurnAngle(90);
                    step = 2;
                    break;
                case 2:
                    if( Math.abs( mmCompass.GetCurrentAngle() - mmCompass.GetTarget() ) < 5) {
                        step = 3;
                    } else {
                        DoTurn();
                    }
                    break;
                case 3:
                    goStraight("3rd Move", Global.dist2Time);
                    step = 0;
                    break;
            }

            robot.dashboard.displayPrintf(2, "Step: " + step);
            robot.dashboard.displayPrintf(10, "Compass Says: " + Global.compass);

            robot.waitForTick(10);

        }
    }

    public  void DoTurn() {

        rpid.Compute();
        rpid.setInput(mmCompass.GetCurrentAngle());

        mechDrive.Drive(0, 0, rpid.GetOutput(), false);


    }

    public void TurnAngle(double angle) {

        mmCompass.SetTargetDegrees(Global.compass + angle);

        rpid.setSetpoint(mmCompass.GetTarget());




    }

}
