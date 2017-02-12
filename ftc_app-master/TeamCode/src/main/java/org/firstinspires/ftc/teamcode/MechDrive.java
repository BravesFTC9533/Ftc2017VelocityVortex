package org.firstinspires.ftc.teamcode;

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
    private static double MIN_POWER = 0.0;

    double timePerRotationClockwiseMS = 4 * 1000.0;
    double timePerRotationCounterClockwiseMS = 4.1 * 1000.0;


    private Gamepad gamepad;
    private HalDashboard hal;



    public MechDrive(Hardware9533 hardware) {
        this.hardware = hardware;
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

    public void Turn(int angle){

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
