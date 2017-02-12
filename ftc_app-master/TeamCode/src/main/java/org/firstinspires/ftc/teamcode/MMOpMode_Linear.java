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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import hallib.HalDashboard;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public abstract class  MMOpMode_Linear extends LinearOpMode { //extends VisionOpMode {

    protected Hardware9533 robot = new Hardware9533();

    private static HalDashboard dashboard = null;
    protected MechDrive mechDrive;




    /**
     * This method returns a global dashboard object for accessing the dashboard on the Driver Station.
     *
     * @return dashboard object.
     */
    public static HalDashboard getDashboard()
    {
        return dashboard;
    }   //getDashboard

    public MMOpMode_Linear() {

    }


    //@Override
    public void runOpMode() throws InterruptedException{
        dashboard = new HalDashboard(telemetry);
        robot.init(hardwareMap);

        mechDrive = new MechDrive(robot);


    }
//
//    public void prepShooter (int targetRPM) {
//        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//        robot.shooterMotor.setMaxSpeed( (targetRPM * 28) / 60);
//
//    }


//    public void turnOnShooter (){
//
//        double power = 0.4;
//        while(power < 1){
//
//            power += 0.005;
//            power = Range.clip(power, 0, 1);
//            robot.shooterMotor.setPower(power);
//
//            if(power == 1) {
//                break;
//            }
//            robot.dashboard.displayPrintf(2, "Waiting for shooter power: %2.5f", power);
//            waitFor(0.02);
//        }
//
//    }

    protected void waitFor(double seconds){
        robot.waitForTick((long)(seconds * 1000));
    }


    /*
    private final ElapsedTime timer = new ElapsedTime();
    private Threader threader = null;
    private Thread thread = null;
    private volatile boolean opModeStarted = false;
    private Mat rgba;
    private Mat gray;
    private boolean hasNewFrame = false;


    @Override
    public final Mat frame(Mat rgba, Mat gray) {
        if (!opModeStarted) return rgba;
        this.rgba = super.frame(rgba, gray);
        Imgproc.cvtColor(rgba, this.gray, Imgproc.COLOR_RGBA2GRAY);
        hasNewFrame = true;
        return rgba;
    }

    public final Mat getFrameRgba() {
        return rgba;
    }

    public final Mat getFrameGray() {
        return gray;
    }

    public boolean hasNewFrame() {
        return hasNewFrame;
    }

    public void discardFrame() {
        hasNewFrame = false;
    }

    //public abstract void runOpMode() throws InterruptedException;

    public final void waitForVisionStart() throws InterruptedException {
        //Give some status info
        //telemetry.addData("Vision Status", "Initializing...\r\n" +
        //      "Please wait, do not stop the OpMode.");

        while (!this.isInitialized()) {
            synchronized (this) {
                this.wait();
            }
        }
    }

    public synchronized void waitForStart() throws InterruptedException {
        while (!this.opModeStarted) {
            synchronized (this) {
                this.wait();
            }
        }
    }

    public void waitOneFullHardwareCycle() throws InterruptedException {
        this.waitForNextHardwareCycle();
        Thread.sleep(1L);
        this.waitForNextHardwareCycle();
    }

    private void waitForNextHardwareCycle() throws InterruptedException {
        synchronized (this) {
            this.wait();
        }
    }

    public void sleep(long milliseconds) throws InterruptedException {
        Thread.sleep(milliseconds);
    }

    public boolean opModeIsActive() {
        return this.opModeStarted;
    }

    @Override
    public final void init() {
        super.init();
        hasNewFrame = false;
        this.rgba = Color.createMatRGBA(width, height);
        this.gray = Color.createMatGRAY(width, height);
        this.threader = new Threader(this);
        this.thread = new Thread(this.threader, "Linear OpMode Helper");
        this.thread.start();
    }

    @Override
    public final void init_loop() {
        this.notifyOrThrowError();
    }

    @Override
    public final void start() {
        this.opModeStarted = true;
        synchronized (this) {
            this.notifyAll();
        }
    }

    @Override
    public final void loop() {
        super.loop();
        this.notifyOrThrowError();
    }

    @Override
    public final void stop() {
        super.stop();
        this.opModeStarted = false;
        this.rgba.release();
        this.gray.release();

        if (!this.threader.isReady()) {
            this.thread.interrupt();
        }

        this.timer.reset();

        while (!this.threader.isReady() && this.timer.time() < 0.5D) {
            Thread.yield();
        }

        if (!this.threader.isReady()) {
            RobotLog.e("*****************************************************************");
            RobotLog.e("User Linear Op Mode took too long to exit; emergency killing app.");
            RobotLog.e("Possible infinite loop in user code?");
            RobotLog.e("*****************************************************************");
            System.exit(-1);
        }
    }

    private void notifyOrThrowError() {
        if (this.threader.hasError()) {
            throw this.threader.getLastError();
        } else {
            synchronized (this) {
                this.notifyAll();
            }
        }
    }

    private static class Threader implements Runnable {
        private final MMOpMode_Linear opModeReference;
        private RuntimeException lastError = null;
        private boolean ready = false;

        public Threader(MMOpMode_Linear var1) {
            this.opModeReference = var1;
        }

        public void run(){
            this.lastError = null;
            this.ready = false;

            try {
                this.opModeReference.runOpMode();
            } catch (InterruptedException var6) {
                RobotLog.d("LinearOpMode received an Interrupted Exception; shutting down this linear op mode");
            } catch (RuntimeException var7) {
                this.lastError = var7;
            } finally {
                this.ready = true;
            }

        }

        public boolean hasError() {
            return this.lastError != null;
        }

        public RuntimeException getLastError() {
            return this.lastError;
        }

        public boolean isReady() {
            return this.ready;
        }
    }*/
}
