package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Util.Helpers;

/**
 * Created by User on 01/14/2017.
 */

public class MechDrive {

    Hardware9533 hardware;

    public MechDrive(Hardware9533 hardware) {
        this.hardware = hardware;
    }



    public void DriveMech(double h, double v, double r) {
        DriveMech(h, v, r, true);
    }

    /***********************************************************************************************/
    public void DriveMech(double h, double v, double r, boolean scalePower)
    {

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

        hardware.dashboard.displayPrintf(3, "Mech Power: " + frontLeft);


        // set power
        hardware.leftMotor.setPower(frontLeft);
        hardware.rightMotor.setPower(frontRight);
        hardware.backRightMotor.setPower(backRight);
        hardware.backLeftMotor.setPower(backLeft);

    }


    //public void Rotate
}
