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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MMOpMode_Linear;
import org.firstinspires.ftc.teamcode.MinionsGyro;
import org.firstinspires.ftc.teamcode.RollingAvg;

import hallib.HalDashboard;

import static java.lang.Math.abs;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tele: Main Op (USE THIS)", group="9533")
@Disabled
public class Minions9533_Teleop extends MMOpMode_Linear {

    /* Declare OpMode members. */
    //Hardware9533   robot           = new Hardware9533();              // Use a K9'shardware

    private static final boolean USE_GYRO = false;
    private static final int tickInterval = 20;


    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    MinionsGyro gyro = null;

    private HalDashboard dashboard;

    private double shooterPower = 1;
    static final double SHOOTER_POWER_INCREMENT  = 0.02;
    static int targetRPM = 2400;
    static final int targetIncrement = 25;

    private boolean currentButtonState = false;
    private boolean previousButtonState = false;

    private boolean leftPressed = false;
    private boolean rightPressed = false;


    int lastPos = 0;


    RollingAvg rollingAverage = null;


    private void handleIntake(){
        if(gamepad2.right_bumper) {
            robot.intake.setPower(1);

        } else if(gamepad2.left_bumper) {
            robot.intake.setPower(-1);

        } else {
            robot.intake.setPower(0);

        }
    }
    private void handleElevator(){
        if(gamepad2.right_bumper) {
            robot.ElevatorLiftBalls();
            //robot.elevator.setPower(1);
        } else if(gamepad2.left_bumper) {
            robot.ElevatorDropBalls();
            //robot.elevator.setPower(-1);
        } else {
            robot.ElevatorStop();
        }
    }


    double shooterAvg = 0;
    double lastTime = 0;
    int counter = 0;
    private void calcAvgRPM(){


        //robot.dashboard.displayPrintf(11, "Time: %s", timer.seconds());

        int currentPos = robot.shooterMotor.getCurrentPosition();
        double currentTime = System.nanoTime();
        if(counter > 0) {


            int delta = abs(currentPos - lastPos);
            double deltaT = (currentTime - lastTime) / ElapsedTime.MILLIS_IN_NANO;

            //robot.dashboard.displayPrintf(12, "Encoder Delta: %s", delta);
            //robot.dashboard.displayPrintf(13, "Time Delta: %s", deltaT);

            double rpm = (delta * ((1000/deltaT) * 60)) / 28;
            //robot.dashboard.displayPrintf(14, "Instant RPM: %s", rpm);

            rollingAverage.add(rpm);

            shooterAvg = rollingAverage.getAverage();
//            int index = count % 50;
//            armp[index] = rpm;
//
//            int avg = 0;
//            if (count > 50) {
//
//                for (int i = 0; i < 50; i++) {
//                    avg += armp[i];
//                }
//                avg /= 50;
//
//            }
//            shooterAvg = avg;
        }

        lastTime = currentTime;
        lastPos = currentPos;
        timer2.reset();
        counter ++;
    }



    double currentPower = 0;
    private void handleShooter() {

        //currentPower = robot.shooterMotor.getPower();
        if(gamepad2.a) {

            if(currentPower == 0) {
                currentPower = 0.4;
            } else if(currentPower < 1) {
                currentPower += 0.005;
            }

            currentPower = Range.clip(currentPower, 0, 1);


            robot.shooterMotor.setPower(currentPower);

            //robot.shooterMotor.setPower(shooterPower);
            //robot.shooterRight.setPower(shooterPower);
        } else {
            //robot.shooterRight.setPower(0);
            robot.shooterMotor.setPower(0);
            currentPower = 0;
        }

        //robot.dashboard.displayPrintf(5, "Shooter: %.2f", currentPower);
        //telemetry.addData("Shooter", shooterPower);
    }

    private void handleShooterSpeed() {
        if(gamepad2.dpad_down) {
            shooterPower -= SHOOTER_POWER_INCREMENT;
        }
        if(gamepad2.dpad_up){
            shooterPower += SHOOTER_POWER_INCREMENT;
        }

        shooterPower = Range.clip(shooterPower, 0,1);


    }


    private void handleTargetRPM() {

        if(leftPressed && gamepad2.dpad_left){
            return;
        }
        if(rightPressed && gamepad2.dpad_right) {
            return;
        }






        if(gamepad2.dpad_left) {
            targetRPM -= targetIncrement;
            leftPressed = true;
        } else {
            leftPressed = false;
        }
        if(gamepad2.dpad_right){
            targetRPM += targetIncrement;
            rightPressed = true;
        } else {
            rightPressed = false;
        }

        targetRPM = Range.clip(targetRPM, 0,6000);


    }

    @Override
    public void runOpMode() {

        super.runOpMode();

        if(USE_GYRO) {
            gyro = new MinionsGyro(robot, "gyro");

        }
        //robot.dashboard.displayPrintf(1, "Hello Driver");

        // Wait for the game to start (driver presses PLAY)

        if(USE_GYRO) {
            //robot.dashboard.displayPrintf(2, "Calibrating..");
            gyro.calibrateGyro();

        }

        //robot.dashboard.displayPrintf(2, "Waiting for start..");
        waitForStart();


        //robot.dashboard.clearDisplay();
        // run until the end of the match (driver presses STOP)

        timer.reset();

        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //long start = System.currentTimeMillis()/60000;
        //long next;


        rollingAverage = new RollingAvg(50);
        while (opModeIsActive()) {

            handleTargetRPM();
            int targetSpeed = (targetRPM * 28) / 60;

            robot.shooterMotor.setMaxSpeed(targetSpeed);

            calcAvgRPM();

            //robot.dashboard.displayPrintf(9, "TargetRPM: %s", targetRPM);
            //robot.dashboard.displayPrintf(10, "RPM: %s", shooterAvg);




            currentButtonState = gamepad1.right_bumper;

            if(currentButtonState != previousButtonState) {
                if(currentButtonState) {
                    robot.invertedDrive = !robot.invertedDrive;
                }
                previousButtonState = currentButtonState;
            }

            //robot.dashboard.displayPrintf(1, "Drive Mode:");
            if(robot.invertedDrive) {
                //robot.dashboard.displayPrintf(2, "Reverse");
            } else {
                //robot.dashboard.displayPrintf(2, "Normal");
            }



            handleIntake();
            handleElevator();
            handleShooter();
            //handleShooterSpeed();

            //robot.DriveRobot(-gamepad1.left_stick_y, -gamepad1.right_stick_y);


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(tickInterval);
        }

    }




}