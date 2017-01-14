package org.firstinspires.ftc.teamcode;

import android.view.animation.RotateAnimation;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import hallib.HalDashboard;

import static java.lang.Math.abs;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Hardware9533
{

    public final static double ACCEL_RATE = 0.2;

    private final static double MAX_SPEED = 0.8;

    public boolean invertedDrive = false;

    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  backLeftMotor = null;
    public DcMotor  backRightMotor = null;

    public DcMotor elevator = null;
    public DcMotor intake = null;

    public DcMotor shooterMotor = null;


    public HiTechnicNxtGyroSensor gyro = null;

    public HalDashboard dashboard;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware9533() {


    }

//
//    static final double     COUNTS_PER_MOTOR_REV    = 21000 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        dashboard = MMOpMode_Linear.getDashboard();

        // save reference to HW Map
        hwMap = ahwMap;

        // Define gyro
        //gyro = (HiTechnicNxtGyroSensor)hwMap.gyroSensor.get("gyro");


        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left");
        rightMotor  = hwMap.dcMotor.get("right");
        backLeftMotor = hwMap.dcMotor.get("backLeft");
        backRightMotor = hwMap.dcMotor.get("backRight");


        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hwMap.dcMotor.get("ballGrabber");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = hwMap.dcMotor.get("elevator");
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);



        shooterMotor = hwMap.dcMotor.get("shooterMotor");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        intake.setPower(0);
        elevator.setPower(0);

        shooterMotor.setPower(0);
        //shooterRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void Stop(){
        //AccelerateMotor(this.leftMotor, 0);
        //AccelerateMotor(this.rightMotor, 0);

        this.leftMotor.setPower(0);
        this.rightMotor.setPower(0);
        this.backLeftMotor.setPower(0);
        this.backRightMotor.setPower(0);
    }


    public void DriveMech(double h, double v, double r) {
        DriveMech(h, v, r, true);
    }

    /***********************************************************************************************/
    public void DriveMech(double h, double v, double r, boolean scalePower)
    {

        // invert drive!
        if(this.invertedDrive) {
            h*=-1;
            v*=-1;
        }

        // make sure values are inside valid motor range
        h = clipMotorPower(h);
        v = clipMotorPower(v);
        r = clipMotorPower(r);

        // scale inputs for easier control at lower speeds
        if(scalePower) {
            h = scale(h);
            v = scale(v);
            r = scale(r);
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
            frontLeft = scalePower(frontLeft, max);
            frontRight = scalePower(frontRight, max);
            backLeft = scalePower(backLeft, max);
            backRight = scalePower(backRight, max);
        }


        // write power to dashboard
        dashboard.displayPrintf(6, "H: " + String.valueOf(h));
        dashboard.displayPrintf(7, "V: " + String.valueOf(v));
        dashboard.displayPrintf(8, "R: " + String.valueOf(r));

        dashboard.displayPrintf(3, "Mech Power: " + frontLeft);


        // set power
        leftMotor.setPower(frontLeft);
        rightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
        backLeftMotor.setPower(backLeft);

    }

    public void Auto_Mech(double h, double v, double r)
    {
        h = clipMotorPower(h);
        v = clipMotorPower(v);
        r = clipMotorPower(r);

        // scale inputs for easier control at lower speeds
        h = scale(h);
        v = scale(v);
        r = scale(r);


        // add vectors
        double frontLeft =  v-h+r;
        double frontRight = v+h-r;
        double backRight =  v-h-r;
        double backLeft =   v+h+r;

        dashboard.displayPrintf(6, "H: " + String.valueOf(h));
        dashboard.displayPrintf(7, "V: " + String.valueOf(v));
        dashboard.displayPrintf(8, "R: " + String.valueOf(r));

        leftMotor.setPower(frontLeft);
        rightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
        backLeftMotor.setPower(backLeft);
    }

    private double scale(double power){
        int modifier = 1;

        if (power == 0 )
        {
            return 0;
        }

        if(power < 0){
            modifier *= -1;
        }

        return  (power * power * modifier);
    }

    public void DriveRobot(double leftPower, double rightPower) {
        double left = 0;
        double right = 0;
        if(this.invertedDrive) {
            //reverse all the things
            left = AccelerateMotor(this.rightMotor, -leftPower);
            right = AccelerateMotor(this.leftMotor, -rightPower);
        } else {
            left = AccelerateMotor(this.leftMotor, leftPower);
            right = AccelerateMotor(this.rightMotor, rightPower);
        }
        dashboard.displayPrintf(3, "left: %.2f", left);
        dashboard.displayPrintf(4, "right: %.2f", right);
    }

    public double AccelerateMotor(DcMotor motor, double targetPower) {
        targetPower = Range.clip(targetPower, -MAX_SPEED, MAX_SPEED);

        double power = this.accel(motor.getPower(), targetPower, ACCEL_RATE);
        motor.setPower(power);
        return power;

    }

    public void ElevatorLiftBalls(){
        this.elevator.setPower(-0.75);
    }
    public void ElevatorDropBalls(){
        this.elevator.setPower(1);
    }

    public void ElevatorStop(){
        this.elevator.setPower(0);
    }

    private double accel(double motorPower, double targetPower, double accelRate) {



        if(accelRate < (abs(motorPower - targetPower))){
            if(motorPower - targetPower < 0) {
                return  (motorPower + accelRate);
            } else {
                return motorPower - accelRate;
            }
        } else {
            return  targetPower;
        }
    }




    // Scale motor power based on the max for all wheels
    // 1, 1, 1, 3 will become .33, .33, .33, 1
    private static double scalePower(double value, double max){
        if(max == 0){return  0;}
        return  value / max;
    }

    // motor power clipping helper
    private static double clipMotorPower(double value){
        return Range.clip(value, -1, 1);
    }


}

