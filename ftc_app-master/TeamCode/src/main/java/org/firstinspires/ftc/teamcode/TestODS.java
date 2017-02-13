package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.Helpers;

/**
 * Created by Kerfuffle on 2/12/2017.
 */

@Autonomous(name = "ODS", group = "Test")
public class TestODS extends MMOpMode_Linear {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();

        waitForStart();

        driveTo(36, 0.4, 0.4, 0);

        /*while (opModeIsActive())
        {
            robot.dashboard.displayText(0, "Raw: " + robot.ods.getRawLightDetected());
            robot.dashboard.displayText(1, "Reg: " + robot.ods.getLightDetected());
        }*/
    }


    private void driveTo(double dist, double h, double v, double r)
    {
        int backLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
        int frontLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
        int backRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
        int frontRightTarget = robot.rightMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        h = Helpers.clipMotorPower(h);
        v = Helpers.clipMotorPower(v);
        r = Helpers.clipMotorPower(r);

        // add vectors
        double frontLeft =  v-h+r;
        double frontRight = v+h-r;
        double backRight =  v-h-r;
        double backLeft =   v+h+r;

        // since adding vectors can go over 1, figure out max to scale other wheels
        double max = Math.max(
                Math.abs(backLeft),
                Math.max(
                        Math.abs(backRight),
                        Math.max(
                                Math.abs(frontLeft), Math.abs(frontRight)
                        )
                )
        );

        // only need to scale power if max > 1
        if(max > 1){
            frontLeft = Helpers.scalePower(frontLeft, max);
            frontRight = Helpers.scalePower(frontRight, max);
            backLeft = Helpers.scalePower(backLeft, max);
            backRight = Helpers.scalePower(backRight, max);
        }

        robot.leftMotor.setPower(frontLeft);
        robot.rightMotor.setPower(frontRight);
        robot.backRightMotor.setPower(backRight);
        robot.backLeftMotor.setPower(backLeft);
    }

}
