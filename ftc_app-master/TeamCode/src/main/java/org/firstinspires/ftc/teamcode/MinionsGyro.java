package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;

import hallib.HalDashboard;

import static java.lang.Math.abs;

/**
 * Created by User on 11/05/2016.
 */

public class MinionsGyro {


    long lastTime = 0;

    double gyroHeading = 0;
    double zeroOffset = 0;
    double deadband = 0;

    HiTechnicNxtGyroSensor gyro = null;

    HalDashboard dashboard = null;
    Hardware9533 robot = null;

    public MinionsGyro(Hardware9533 robot, String name){

        this.robot = robot;
        this.gyro = (HiTechnicNxtGyroSensor)robot.hwMap.gyroSensor.get(name);
        this.dashboard = robot.dashboard;
    }

    public void reset() {
        this.gyroHeading = 0;
    }

    public double getHeading() {
        integrateGyro();

        double heading =gyroHeading * 1000;
        if(abs(heading) > 400) {

            calibrateGyro();

        }
        return  heading;
    }

    public void calibrateGyro()  {

        dashboard.displayText(5, "Calibrating gyro...");


        double value = gyro.getRotationFraction();
        double minValue = value;
        double maxValue = value;
        double sum = 0.0;
        for (int i = 0; i < 50; i++) {
            value = gyro.getRotationFraction();
            sum += value;
            if (value < minValue) minValue = value;
            if (value > maxValue) maxValue = value;
            robot.waitForTick(20);
        }
        zeroOffset = sum / 50.0;
        deadband = maxValue - minValue;

        dashboard.displayText(5, "Calibration complete");
    }

    public void integrateGyro() {
        long currTime = System.currentTimeMillis();
        double value = robot.gyro.getRotationFraction() - zeroOffset;
        if (abs(value) < deadband) value = 0.0;
        gyroHeading += value * (currTime - lastTime) / 1000.0;
        lastTime = currTime;

    }
}
