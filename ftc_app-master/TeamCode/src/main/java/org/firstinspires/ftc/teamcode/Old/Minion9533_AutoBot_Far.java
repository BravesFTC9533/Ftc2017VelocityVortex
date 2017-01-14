/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MMOpMode_Linear;
import org.firstinspires.ftc.teamcode.MinionsGyro;

import hallib.HalDashboard;

import static java.lang.Math.abs;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autobot Far", group="Pushbot")
@Disabled
public class Minion9533_AutoBot_Far extends MMOpMode_Linear {

    /* Declare OpMode members. */
                                                               // could also use HardwarePushbotMatrix class.
    private ElapsedTime runtime = new ElapsedTime();

    //private HalDashboard dashboard;
    MinionsGyro gyro = null;


    static final double     FORWARD_SPEED = 0.4;
    static final double     TURN_SPEED    = 0.6;

    static int targetRPM = 2400;

    static final double DEGREES_PER_SECOND = 108;

    private void goStraight(String step, double time){
        //robot.DriveRobot(FORWARD_SPEED, FORWARD_SPEED, telemetry);

        //dashboard.displayPrintf(0, "%s: Drive straight for %s second(s)", step, time);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
            //dashboard.displayPrintf(1, "Straight: %2.5f S Elapsed: %s", runtime.seconds(), robot.leftMotor.getCurrentPosition());

            //robot.DriveRobot(FORWARD_SPEED, FORWARD_SPEED);
            robot.waitForTick(40);
        }

        //robot.Stop();
        pauseBetweenSteps();
    }




    private boolean inRange(double target, double actual, int range) {

        if(abs(actual) - range < target && abs(actual) + range > target ) {
            return true;
        }
        return false;
    }

    private void turnLeft(String step, int degrees){

        //dashboard.displayPrintf(0, "%s: Turn Left %s degrees", step, degrees);
        gyro.reset();


        while(opModeIsActive()) {
            double heading = gyro.getHeading();
            //dashboard.displayPrintf(6, "Heading: %.2f", heading);
            if(inRange(degrees, heading , 5)) {
                break;
            }
            //robot.DriveRobot(-TURN_SPEED, TURN_SPEED);

            robot.waitForTick(10);
        }
        //robot.Stop();

        pauseBetweenSteps();


    }

    private void turnRight(String step, int degrees){

        //dashboard.displayPrintf(0, "%s: Turn Right %s degrees", step, degrees);
        gyro.reset();


        while(opModeIsActive()) {
            double heading = gyro.getHeading();
            //dashboard.displayPrintf(6, "Heading: %.2f", heading);
            if(inRange(degrees, heading, 5)) {

                break;
            }
            //robot.DriveRobot(TURN_SPEED, -TURN_SPEED);

            robot.waitForTick(10);
        }

        //robot.Stop();
        pauseBetweenSteps();
    }


    private void shootBalls() {


        //robot.dashboard.displayText(3, "Turning on elevator");
        robot.ElevatorLiftBalls();
        //robot.dashboard.displayText(4, "Waiting for 0.5 seconds");
        waitFor(0.5);
        //robot.dashboard.displayText(4, "Stopping elevator. let wheels spin back up");
        robot.ElevatorStop();
        //robot.dashboard.displayText(4, "Waiting for 1 second");
        waitFor(4);

        //robot.dashboard.displayText(4, "Turning on elevator");
        robot.ElevatorLiftBalls();
        //robot.dashboard.displayText(4, "Waiting for 1 second");
        waitFor(1);


        robot.shooterMotor.setPower(0);
        robot.ElevatorStop();


    }


    @Override
    public void runOpMode() {

       super.runOpMode();


        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.shooterMotor.setMaxSpeed( (targetRPM * 28) / 60);

        //gyro = new MinionsGyro(robot, "gyro");
        //gyro.calibrateGyro();



        //dashboard = getDashboard();
        // Send telemetry message to signify robot waiting;



        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        waitFor(0.1);
        //robot.dashboard.displayText(0, "Moving Forward half block");
        goStraight("Step 1", 1.1);
        pauseBetweenSteps();


        //robot.dashboard.displayText(1, "Turning on shooter");

        double power = 0.4;
        while(power < 1){

            power += 0.005;
            power = Range.clip(power, 0, 1);
            robot.shooterMotor.setPower(power);

            if(power == 1) {
                break;
            }
            //robot.dashboard.displayPrintf(2, "Waiting for shooter power: %2.5f", power);
            waitFor(0.02);
        }



        pauseBetweenSteps();
        pauseBetweenSteps();

        shootBalls();

        pauseBetweenSteps();


        goStraight("Push Ball", 1.5);
        //turnRight("Turn to square", 45);
        //goStraight("Park", .5);

        //robot.Stop();

    }


    private void pauseBetweenSteps(){
        logPath("pausing waiting 2 seconds");
        waitFor(1);
    }


    private void logPath(String msg){

        //robot.dashboard.displayText(1, msg);

    }

    private void waitFor(double seconds){
        robot.waitForTick((long)(seconds * 1000));
    }
}
