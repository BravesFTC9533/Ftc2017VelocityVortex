package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Helpers;

import hallib.HalDashboard;

/**
 * Created by User on 01/14/2017.
 */

public class MechDrive {

    private ElapsedTime runtime = new ElapsedTime();
    private Hardware9533 hardware;
    private LinearOpMode opMode;
    private static double MIN_POWER = 0.0;

    double timePerRotationClockwiseMS = 4 * 1000.0;
    double timePerRotationCounterClockwiseMS = 4.1 * 1000.0;


    private Gamepad gamepad;
    private HalDashboard hal;




    public MechDrive(Hardware9533 hardware, LinearOpMode opMode) {
        this.hardware = hardware;
        this.opMode = opMode;
    }



    public void Drive(double h, double v, double r) {
        Drive(h, v, r, true);
    }

    public void setGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }
    public void setHalBoard(HalDashboard hal)
    {
        this.hal = hal;
    }

    /***********************************************************************************************/
    public void Drive(double h, double v, double r, boolean scalePower) {
        if (gamepad != null)
        {
            if (gamepad.left_trigger > 0.8)
            {
                MIN_POWER = 0;
                hal.displayText(3, "Im triggered");
            }
            else
            {
                MIN_POWER = 0.1;
                hal.displayText(3, "");
            }
        }

        if(Math.abs(h) < MIN_POWER) {
            h = 0;
        }
        if(Math.abs(v) < MIN_POWER) {
            v = 0;
        }
//        if(Math.abs(r) < MIN_POWER) {
//            r = 0;
//        }

        // invert drive!
        if(hardware.invertedDrive) {
            h*=-1;
            v*=-1;
        }

        // make sure values are inside valid motor range
        h = Helpers.clipMotorPower(h);
        v = Helpers.clipMotorPower(v);
        r = Helpers.clipMotorPower(r);

        // scale inputs for easier control at lower speeds
        if(scalePower) {
            h = Helpers.scale(h);
            v = Helpers.scale(v);
            r = Helpers.scale(r);
        }

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


        // write power to dashboard
        hardware.dashboard.displayPrintf(6, "H: " + String.valueOf(h));
        hardware.dashboard.displayPrintf(7, "V: " + String.valueOf(v));
        hardware.dashboard.displayPrintf(8, "R: " + String.valueOf(r));

        //hardware.dashboard.displayPrintf(3, "Mech Power: " + frontLeft);


        // set power
        hardware.leftMotor.setPower(frontLeft);
        hardware.rightMotor.setPower(frontRight);
        hardware.backRightMotor.setPower(backRight);
        hardware.backLeftMotor.setPower(backLeft);

    }

    public void TurnByTime(double angle){

        double timeForTurn = 0.0;

        if(angle < 0) {
            timeForTurn = (timePerRotationCounterClockwiseMS / 360) * Math.abs(angle);

        } else {
            timeForTurn = (timePerRotationClockwiseMS / 360) * Math.abs(angle);
        }


        hardware.dashboard.displayPrintf(15, "Time for turn: %f", timeForTurn);
        double rotationSpeed = 0.3;


        if(angle < 0) {
            rotationSpeed *= -1;
        }

        runtime.reset();

        Drive(0, 0, rotationSpeed);
        while(runtime.milliseconds() < timeForTurn) {
            hardware.dashboard.displayPrintf(14, "Runtime: %f", runtime.milliseconds());
        }

        Stop();
    }

    public void turn(double angle) {

        double ticksPerDegree_bl = 26.36111111;
        double ticksPerDegree_br = 26.38277778;

        double ticksPerDegree_fl = 26.41777778;
        double ticksPerDegree_fr = 26.40777778;



        double target_bl = Math.abs(ticksPerDegree_bl*angle);
        double target_br = Math.abs(ticksPerDegree_br*angle);
        double target_fl = Math.abs(ticksPerDegree_fl*angle);
        double target_fr = Math.abs(ticksPerDegree_fr*angle);

        double turnPower = 0.9;

        //right negative is pos angle;




        hardware.resetPosition();

        boolean rightIsNegative = angle > 0;

        double leftPower = rightIsNegative ? turnPower : turnPower * -1;
        double rightPower = rightIsNegative ? turnPower*-1 : turnPower;



        double flp = hardware.leftMotor.getPosition();
        double frp = hardware.rightMotor.getPosition();
        double blp = hardware.backLeftMotor.getPosition();
        double brp = hardware.backRightMotor.getPosition();

        hardware.dashboard.displayPrintf(9, "FL: t: %.3f, p: %.2f, pow: %.2f", target_fl, flp, leftPower);
        hardware.dashboard.displayPrintf(10, "FR: t: %.3f, p: %.2f, pow: %.2f", target_fr, frp, rightPower);

        hardware.dashboard.displayPrintf(12, "BL: t: %.3f, p: %.2f, pow: %.2f", target_bl, blp, leftPower);
        hardware.dashboard.displayPrintf(13, "BR: t: %.3f, p: %.2f, pow: %.2f", target_br, brp, rightPower);



        hardware.dashboard.displayPrintf(14, "about to run");



        ElapsedTime time = new ElapsedTime();


        //hardware.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //hardware.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);


        if(rightIsNegative) {
            target_br *= -1;
            target_fr *= -1;
        } else {
            target_bl *= -1;
            target_fl *= -1;
        }
        hardware.rightMotor.runToPosition((int)target_fr);
        hardware.leftMotor.runToPosition((int)target_fl);
        hardware.backRightMotor.runToPosition((int)target_br);
        hardware.backLeftMotor.runToPosition((int)target_bl);


        hardware.leftMotor.setPower(leftPower);
        hardware.backLeftMotor.setPower(leftPower);
        hardware.rightMotor.setPower(rightPower);
        hardware.backRightMotor.setPower(rightPower);


        while(opMode.opModeIsActive() && IsBusy() && time.seconds() < 5){

            flp = hardware.leftMotor.getPosition();
            frp = hardware.rightMotor.getPosition();
            blp = hardware.backLeftMotor.getPosition();
            brp = hardware.backRightMotor.getPosition();


            hardware.dashboard.displayPrintf(14, "running");
            hardware.dashboard.displayPrintf(9, "FL: t: %.3f, p: %.2f, pow: %.2f", target_fl, flp, leftPower);
            hardware.dashboard.displayPrintf(10, "FR: t: %.3f, p: %.2f, pow: %.2f", target_fr, frp, rightPower);

            hardware.dashboard.displayPrintf(12, "BL: t: %.3f, p: %.2f, pow: %.2f", target_bl, blp, leftPower);
            hardware.dashboard.displayPrintf(13, "BR: t: %.3f, p: %.2f, pow: %.2f", target_br, brp, rightPower);

        }


        hardware.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Stop();

//        while (opMode.opModeIsActive()
//                && Math.abs(flp) <= target_fl
//                && Math.abs(frp) <= target_fr
//                && Math.abs(blp) <= target_bl
//                && Math.abs(brp) <= target_br
//                && time.seconds() < 5
//                ) {
//
//
//
//            flp = hardware.leftMotor.getPosition();
//            frp = hardware.rightMotor.getPosition();
//            blp = hardware.backLeftMotor.getPosition();
//            brp = hardware.backRightMotor.getPosition();
//
//            hardware.dashboard.displayPrintf(14, "running");
//            hardware.dashboard.displayPrintf(9, "FL: t: %.3f, p: %.2f, pow: %.2f", target_fl, flp, leftPower);
//            hardware.dashboard.displayPrintf(10, "FR: t: %.3f, p: %.2f, pow: %.2f", target_fr, frp, rightPower);
//
//            hardware.dashboard.displayPrintf(12, "BL: t: %.3f, p: %.2f, pow: %.2f", target_bl, blp, leftPower);
//            hardware.dashboard.displayPrintf(13, "BR: t: %.3f, p: %.2f, pow: %.2f", target_br, brp, rightPower);
//
//
//            hardware.leftMotor.setPower(leftPower);
//            hardware.backLeftMotor.setPower(leftPower);
//            hardware.rightMotor.setPower(rightPower);
//            hardware.backRightMotor.setPower(rightPower);
//
//
//        }
//        Stop();

        hardware.dashboard.displayPrintf(14, "Stopped");



    }


    public boolean IsBusy() {
        return hardware.leftMotor.motor.isBusy()
                || hardware.rightMotor.motor.isBusy()
                || hardware.backLeftMotor.motor.isBusy()
                || hardware.backRightMotor.motor.isBusy();

    }

    public void Stop() {

        hardware.leftMotor.setPower(0);
        hardware.rightMotor.setPower(0);
        hardware.backRightMotor.setPower(0);
        hardware.backLeftMotor.setPower(0);

        // write power to dashboard
        hardware.dashboard.displayPrintf(6, "H: " + String.valueOf(0));
        hardware.dashboard.displayPrintf(7, "V: " + String.valueOf(0));
        hardware.dashboard.displayPrintf(8, "R: " + String.valueOf(0));
    }

    //public void Rotate
}
