package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Util.Helpers;

/**
 * Created by Kerfuffle on 2/12/2017.
 */

@Autonomous(name = "ODS", group = "Test")
public class TestODS extends MMOpMode_Linear {

    static final int MAX_ROBOT_SPEED = 3500;
    double ENCODER_Y_INCHES_PER_COUNT = 0.008726388889;
    double ENCODER_X_INCHES_PER_COUNT = 0.007953739871; //0.008180989581; // 0.007817390046;
    static final float      MM_PER_INCH             = 25.4f;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private final double TARGET_LIGHT = 0.4;

    private enum Direction
    {
        LEFT, RIGHT;
    }
    private enum TeamColor
    {
        RED, BLUE;
    }

    TeamColor teamColor = TeamColor.RED;
    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();

        Direction lastDir = Direction.RIGHT;


        waitForStart();


        double length = teamColor == TeamColor.BLUE ? 60 : 44;
        driveTo(length, 0, -1, 0);
        //driveTo(length, -0.7, -0.7, 0);
        waitFor(0.1);
        mechDrive.turn(45, 1);

        driveTo(2, -0.5, 0, 0);

        while (opModeIsActive())
        {

            if (robot.ods.getLightDetected() > TARGET_LIGHT)
            {
                // on line
                // go forward

                driveTo(5, -1, 0, 0);
            }
            else
            {
                // not on line

                if (lastDir == Direction.LEFT)
                {
                    driveTo(1, -0.5, 0, 0);
                    lastDir = Direction.RIGHT;
                    // go right
                }
                else if (lastDir == Direction.RIGHT)
                {
                    driveTo(1, 0.5, 0, 0);
                    lastDir = Direction.LEFT;
                    // go left
                }
            }

            robot.dashboard.displayText(1,"" + robot.ods.getLightDetected());
        }
    }


    private void scaleDrive(int target, double h, double v) {

        if(h == 0 && v != 0) {
            scaleDriveV(target, v);
        } else if(v == 0 && h != 0) {
            scaleDriveH(target, h);
        } else {
            mechDrive.Drive(h, v, 0, false);
        }

    }


    private double getScalePower(double target, double pos, double power) {


        int encoderPositionAtFullSpeed = 250;

        //ramp up speed
        if(pos < encoderPositionAtFullSpeed) {
            if(power > 0) {
                return Range.clip(pos / encoderPositionAtFullSpeed, 0.10, power);
            } else {
                return Range.clip((pos / encoderPositionAtFullSpeed) * -1, -1, -0.10);
            }
        }


        //now that we've ramped up, ramp down towards end
        double percent = 0.0 ;


        if(pos > 0) {
            percent = pos / target;
        }



        // from 80% to 100%, slowly ramp down
        //


        if(percent >= 0.8) {


            double val = (1-percent) * 5; // scale last 20% up to 100%

            if(power > 0) {
                return Range.clip(power * val, 0.3, 1);
            } else {
                return Range.clip((power * val * -1), -1, -0.3);
            }


        } else {
            return power;
        }
    }

    private void scaleDriveV(int target, double v){
        double pos = Math.abs(robot.leftMotor.getPosition());

        double scalePower = getScalePower(target, pos, v);


        mechDrive.Drive(0, scalePower, 0, false);

    }

    private void scaleDriveH(int target, double h){
        double pos = Math.abs(robot.leftMotor.getPosition());

        double scalePower = getScalePower(target, pos, h);

        mechDrive.Drive(scalePower, 0, 0, false);
    }


    private void driveTo(double dist, double h, double v, double r) {

        //21 5/8
        //21 1/2
        //21 1/8

        if(teamColor == TeamColor.BLUE) {
            v *= -1;
        }

//        if(Math.abs(h) > 0 && v == 0) {
//            dist *= 1.11;
//        }

        robot.backLeftMotor.resetPosition();
        robot.backRightMotor.resetPosition();
        robot.leftMotor.resetPosition();
        robot.rightMotor.resetPosition();

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setMaxSpeed(MAX_ROBOT_SPEED);

        if (opModeIsActive())
        {
            int target = 0;

            if(v != 0 && h == 0) {
                target = (int)(dist / ENCODER_Y_INCHES_PER_COUNT);
            } else if(h != 0 && v == 0) {
                target = (int)(dist / ENCODER_X_INCHES_PER_COUNT);
            } else {
                target = (int)(dist*COUNTS_PER_INCH);
            }


            do{
                robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(4, "frontLeft: " + robot.rightMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(5, "frontRight: " + robot.leftMotor.getPosition() + " target: " + target);
                scaleDrive(target, h, v);


            }
            while ( opModeIsActive()
                    && (Math.abs(robot.backLeftMotor.getPosition()) < target)
                    && (Math.abs(robot.backRightMotor.getPosition()) < target)
                    && (Math.abs(robot.rightMotor.getPosition()) < target)
                    && (Math.abs(robot.leftMotor.getPosition()) < target));
            mechDrive.Stop();
        }

    }


}
