/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftclib.FtcOpMode;
import trclib.TrcDbgTrace;
import trclib.TrcDigitalInput;
import trclib.TrcMotor;

/**
 * This class implements the Modern Robotics Motor Controller extending TrcMotor. It provides implementation of the
 * abstract methods in TrcMotor. It supports limit switches. When this class is constructed with limit switches,
 * setPower will respect them and will not move the motor into the direction where the limit switch is activated.
 * It also provides a software encoder reset without switching the Modern Robotics motor controller mode which is
 * problematic.
 */
public class FtcDcMotor
{
    private static final String moduleName = "FtcDcMotor";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;


    private String instanceName;

    public DcMotor motor;
    private int zeroEncoderValue;
    private int prevEncPos;
    private int positionSensorSign = 1;
    private double prevPower = 0.0;
    private boolean softLowerLimitEnabled = false;
    private boolean softUpperLimitEnabled = false;
    private double softLowerLimit = 0.0;
    private double softUpperLimit = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.

     */
    public FtcDcMotor(HardwareMap hardwareMap, String instanceName)
    {

        this.instanceName = instanceName;

        motor = hardwareMap.dcMotor.get(instanceName);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        while(motor.isBusy()) {
//
//        }
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setMaxSpeed(4000);

        zeroEncoderValue = motor.getCurrentPosition();
        prevEncPos = zeroEncoderValue;
    }   //FtcDcMotor

//    /**
//     * Constructor: Create an instance of the object.
//     *
//     * @param instanceName specifies the instance name.
//     * @param lowerLimitSwitch specifies the lower limit switch object.
//     * @param upperLimitSwitch specifies the upper limit switch object.
//     */
//    public FtcDcMotor(String instanceName, TrcDigitalInput lowerLimitSwitch, TrcDigitalInput upperLimitSwitch)
//    {
//        this(FtcOpMode.getInstance().hardwareMap, instanceName, lowerLimitSwitch, upperLimitSwitch);
//    }   //FtcDcMotor
//
//    /**
//     * Constructor: Create an instance of the object.
//     *
//     * @param instanceName specifies the instance name.
//     * @param lowerLimitSwitch specifies the lower limit switch object.
//     */
//    public FtcDcMotor(String instanceName, TrcDigitalInput lowerLimitSwitch)
//    {
//        this(instanceName, lowerLimitSwitch, null);
//    }   //FtcDcMotor



    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    //
    // Implements TrcMotor abstract methods.
    //

    /**
     * This method returns the state of the motor controller direction.
     *
     * @return true if the motor direction is inverted, false otherwise.
     */

    public boolean getInverted()
    {
        final String funcName = "getInverted";
        boolean inverted = motor.getDirection() == DcMotor.Direction.REVERSE;

        return inverted;
    }   //getInverted

    /**
     * This method returns the motor position by reading the position sensor. The position sensor can be an encoder
     * or a potentiometer.
     *
     * @return current motor position.
     */

    public double getPosition()
    {
        final String funcName = "getPosition";
        int currEncPos = motor.getCurrentPosition();



        if (currEncPos == 0 && Math.abs(prevEncPos) > 1000)
        {

            currEncPos = prevEncPos;
        }
        else
        {
            prevEncPos = currEncPos;
        }

        int position = positionSensorSign*(currEncPos - zeroEncoderValue);


        return (double)position;
    }   //getPosition

    /**
     * This method inverts the position sensor direction. This may be rare but there are scenarios where the motor
     * encoder may be mounted somewhere in the power train that it rotates opposite to the motor rotation. This will
     * cause the encoder reading to go down when the motor is receiving positive power. This method can correct this
     * situation.
     *
     * @param inverted specifies true to invert position sensor direction, false otherwise.
     */
    public void setPositionSensorInverted(boolean inverted)
    {
        final String funcName = "setPositionSensorInverted";



        positionSensorSign = inverted? -1: 1;
    }   //setPositionSensorInverted

    /**
     * This method gets the last set power.
     *
     * @return the last setPower value.
     */

    public double getPower()
    {
        final String funcName = "getPower";
        double power = motor.getPower();



        return power;
    }   //getPower

    /**
     * This method returns the battery voltage that powers the motor.
     *
     * @return battery voltage.
     */
    public double getVoltage()
    {
        final String funcName = "getVoltage";
        double voltage = ((ModernRoboticsUsbDcMotorController)motor.getController()).getVoltage();



        return voltage;
    }   //getVoltage



    /**
     * This method resets the motor position sensor, typically an encoder.
     */

    public void resetPosition()
    {
        final String funcName = "resetPosition";



        //
        // Modern Robotics motor controllers supports resetting encoders by setting the motor controller mode. This
        // is a long operation and has side effect of disabling the motor controller unless you do another setMode
        // to re-enable it. For example:
        //      motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //      motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        // It is a lot more efficient doing it in software.
        //
        zeroEncoderValue = motor.getCurrentPosition();
    }   //resetPosition

    /**
     * This method enables/disables motor brake mode. In motor brake mode, set power to 0 would stop the motor very
     * abruptly by shorting the motor wires together using the generated back EMF to stop the motor. When brakMode
     * is false (i.e. float/coast mode), the motor wires are just disconnected from the motor controller so the motor
     * will stop gradually.
     *
     * @param enabled specifies true to enable brake mode, false otherwise.
     */

    public void setBrakeModeEnabled(boolean enabled)
    {
        final String funcName = "setBrakeModeEnabled";


        motor.setZeroPowerBehavior(enabled? DcMotor.ZeroPowerBehavior.BRAKE: DcMotor.ZeroPowerBehavior.FLOAT);
    }   //setBrakeModeEnabled

    /**
     * This method inverts the motor direction.
     *
     * @param inverted specifies true to invert motor direction, false otherwise.
     */

    public void setInverted(boolean inverted)
    {
        final String funcName = "setInverted";


        motor.setDirection(inverted? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
    }   //setInverted

    /**
     * This method sets the output power of the motor controller.
     *
     * @param power specifies the output power for the motor controller in the range of -1.0 to 1.0.
     */

    public void setPower(double power)
    {
        final String funcName = "setPower";



        if (power != prevPower)
        {
            motor.setPower(power);
            prevPower = power;
        }

    }   //setPower




}   //class FtcDcMotor
