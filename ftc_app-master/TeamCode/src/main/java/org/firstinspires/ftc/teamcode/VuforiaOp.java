package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import android.widget.Toast;

import com.qualcomm.ftcrobotcontroller.R;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.Util.Global;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Kerfuffle on 9/24/2016.
 */

import org.firstinspires.ftc.teamcode.Util.ButtonRange;
import org.firstinspires.ftc.teamcode.Util.OCVUtils;
import org.firstinspires.ftc.teamcode.Util.VortexUtils;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;

import ftclib.FtcAndroidAccel;
import trclib.TrcAccelerometer;
import trclib.TrcSensor;

import static org.firstinspires.ftc.teamcode.Util.VortexUtils.BEACON_BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.Util.VortexUtils.BEACON_BLUE_LOW;
import static org.firstinspires.ftc.teamcode.Util.VortexUtils.getImageFromFrame;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "DONT USE ME", group = "vuf")
//@Autonomous(name="Concept: Vuforia Navigation", group =sensorManager = (SensorManager) Activity.getSystemService(SENSOR_SERVICE);"Concept")
public class VuforiaOp extends MMOpMode_Linear{ //extends MMOpMode_Linear{

    public static double beaconStepBack = 2;
    public static int numBeacons = 2;


    double timePerRotationClockwiseMS = 4 * 1000.0;
    double timePerRotationCounterClockwiseMS = 4.1 * 1000.0;


    static int targetRPM = 2400;

    enum TeamColor {
        BLUE,
        RED
    }

    enum Proximity
    {
        NEAR, FAR
    }

    enum States {

        MOVE_FROM_START,
        FIND_BEACON,
        GET_BEACON_LOCATION,
        MOVE_TO_BEACON,
        MOVE_TO_CORRECT_BUTTON,
        FIX_ANGLE,
        PRE_MOVE_IN,
        PRESS_BUTTON,
        PRE_MOVE_OUT,
        BACK_OFF_BUTTON,
        GET_BEACON_COLOR,
        DONE
    }


    private VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();


    VuforiaTrackables beacons;

    public static TeamColor teamColor = TeamColor.RED;
    public static Proximity proximity = Proximity.NEAR;

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AeWceoD/////AAAAGWvk7AQGLUiTsyU4mSW7gfldjSCDQHX76lt9iPO5D8zaboG428rdS9WN0+AFpAlc/g4McLRAQIb5+ijFCPJJkLc+ynXYdhljdI2k9R4KL8t3MYk/tbmQ75st9VI7//2vNkp0JHV6oy4HXltxVFcEbtBYeTBJ9CFbMW+0cMNhLBPwHV7RYeNPZRgxf27J0oO8VoHOlj70OYdNYos5wvDM+ZbfWrOad/cpo4qbAw5iB95T5I9D2/KRf1HQHygtDl8/OtDFlOfqK6v2PTvnEbNnW1aW3vPglGXknX+rm0k8b0S7GFJkgl7SLq/HFNl0VEIVJGVQe9wt9PB6bJuxOMMxN4asy4rW5PRRBqasSM7OLl4W";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);


        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        beacons = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        beacons.activate();
    }

    private VuforiaTrackableDefaultListener getVisibleBeacon() {
        for (VuforiaTrackable b : beacons) {

            VuforiaTrackableDefaultListener beacon = (VuforiaTrackableDefaultListener) b.getListener();

            if(beacon.isVisible()){
                return beacon;
            }

        }
        return null;
    }

    private VuforiaTrackableDefaultListener getBeacon(String name)
    {
        for (VuforiaTrackable b : beacons)
        {
            VuforiaTrackableDefaultListener beacon = (VuforiaTrackableDefaultListener) b.getListener();
            if (b.getName().equalsIgnoreCase(name) && beacon.isVisible())
            {
                return beacon;
            }
        }
        return null;
    }

    private MMTranslation getCurrentLocation(VuforiaTrackableDefaultListener beacon) {
        OpenGLMatrix pose = beacon.getPose();
        if(pose == null)
            return  null;

        VectorF translation = pose.getTranslation();

        return  new MMTranslation(translation);

    }



    public int getBeaconColor(VuforiaTrackableDefaultListener listener) {
        int color = -1;
        boolean usingFrontCamera = true;

        try {
            Image img = getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);

            runtime.reset();

            do {
                color = getBeaconConfig(img, listener, vuforia.getCameraCalibration()); //TODO
            }
            while (opModeIsActive() && runtime.seconds() < 5 && color == -1);



        } catch (InterruptedException e) {
        }
        catch(Throwable e) {
            robot.dashboard.displayText(0, e.getMessage().toString());
        }

        if(usingFrontCamera) {
            if(color == VortexUtils.BEACON_RED_BLUE) {
                color = VortexUtils.BEACON_BLUE_RED;
            } else if(color == VortexUtils.BEACON_BLUE_RED) {
                color = VortexUtils.BEACON_RED_BLUE;
            }
        }

        if (color == VortexUtils.BEACON_BLUE_RED)
        {
            robot.dashboard.displayPrintf(15, "Blue on left, Red on right");
        }
        else if (color == VortexUtils.BEACON_RED_BLUE)
        {
            robot.dashboard.displayPrintf(15, "Red on left, Blue on right");
        }
        else {
            robot.dashboard.displayPrintf(15, "Unable to determine config: %d", color);
            pauseBetweenSteps();
            //pauseBetweenSteps();
            //pauseBetweenSteps();
            //pauseBetweenSteps();

        }

        return color;
    }


    ButtonRange lastTarget = ButtonRange.Unknown();

    private ButtonRange getTargetButton(VuforiaTrackableDefaultListener visibleBeacon){

        //VuforiaTrackableDefaultListener visibleBeacon = getVisibleBeacon();
        ButtonRange targetButton;
        int mycolor = getBeaconColor(visibleBeacon);

        if(mycolor == -1) {
            logState("[GET_BEACON_COLOR] Unable to determine color");

            return ButtonRange.Unknown();

        } else {
            logState("[GET_BEACON_COLOR] Got a beacon color [%d]", mycolor);
            if (mycolor == VortexUtils.BEACON_BLUE_RED) {

                if (teamColor == TeamColor.BLUE) {
                    targetButton = ButtonRange.LeftButton();
                } else {
                    targetButton = ButtonRange.RightButton();
                }
            } else if (mycolor == VortexUtils.BEACON_RED_BLUE) {
                if (teamColor == TeamColor.BLUE) {
                    targetButton = ButtonRange.RightButton();
                } else {
                    targetButton = ButtonRange.LeftButton();
                }

            } else {
                //logState("[GET_BEACON_COLOR] Unable to determine color");

                return ButtonRange.Unknown();
            }
            lastTarget = targetButton;
            return  targetButton;
        }


    }


    private void turn(double angle) {
        double timeForTurn = 0.0;

        if(angle < 0) {
            timeForTurn = (timePerRotationCounterClockwiseMS / 360) * Math.abs(angle);

        } else {
            timeForTurn = (timePerRotationClockwiseMS / 360) * Math.abs(angle);
        }


        robot.dashboard.displayPrintf(15, "Time for turn: %f", timeForTurn);
        double rotationSpeed = 0.3;


        if(angle < 0) {
            rotationSpeed *= -1;
        }

        runtime.reset();
        Drive(0, 0, rotationSpeed);
        while(opModeIsActive() && runtime.milliseconds() < timeForTurn) {
            robot.dashboard.displayPrintf(14, "Runtime: %f", runtime.milliseconds());
        }

        Stop();
    }


    private void fixAngles(VuforiaTrackableDefaultListener visibleBeacon){
        logState("[SQUARE UP TO WALL]");
        //VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation angles = null;
        double angleToWall = 0;


        //visibleBeacon = getVisibleBeacon();
        if(visibleBeacon == null){
            //uh oh
            logState("Unable to locate beacon");
            Stop();
            return;

        }


        angles = anglesFromTarget(visibleBeacon);

        if (angles == null)
        {
            return;
        }

        angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
        double angle = angleToWall - 90;
        logState("[SQUARE UP TO WALL] Angle: %f", angle);

        if(Math.abs(angle) > 4) {
            turn(angle);
        }


    }


    private void moveToBeacon(VuforiaTrackableDefaultListener visibleBeacon) {
        logState("[MOVE_TO_BEACON] Move closer to beacon");

        //VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation angles;
        MMTranslation currentLocation = null;
        double angleToWall;

        angles = anglesFromTarget(visibleBeacon);
        runtime.reset();
        do {
            //visibleBeacon = getVisibleBeacon();
            if(visibleBeacon == null){
                Stop();
                logState("Unable to find beacon");
                return;
            }
            currentLocation = getCurrentLocation(visibleBeacon);

            angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
            moveToBeacon(currentLocation.getX(), currentLocation.getZ(), angleToWall - 90);
        } while(opModeIsActive() && currentLocation.getZ() < -500 && runtime.seconds() < 10);
    }

    private void centerOnBeacon(VuforiaTrackableDefaultListener visibleBeacon) {
        logState("[Center On Beacon] Centering on beacon");

        //VuforiaTrackableDefaultListener visibleBeacon = null;
//        MMTranslation angles;
        MMTranslation currentLocation = null;
//        MMTranslation nav = null;
//
//        double angleToWall;
        runtime.reset();

        do {
            //visibleBeacon = getVisibleBeacon();
            if(visibleBeacon == null){
                mechDrive.Stop();
                logState("Unable to find beacon");
                return;
            }

            //angles = anglesFromTarget(visibleBeacon);


            currentLocation = getCurrentLocation(visibleBeacon);

            double x = currentLocation.getX();
            double moveSpeed = 0.15;

            if(Math.abs(x) >= 20 && opModeIsActive()) {
                if(x < 0) {
                    moveSpeed *= -1;
                }
                mechDrive.Drive(0, moveSpeed, 0, false);
            }

        } while(opModeIsActive() && Math.abs(currentLocation.getX()) < 20 && runtime.seconds() < 2);

        mechDrive.Stop();

    }

    private void moveToCorrectButton(ButtonRange targetButton, VuforiaTrackableDefaultListener visibleBeacon) {


        if (opModeIsActive())
        {
            if (targetButton.getName().equals("Left Button"))
            {
                Drive(0, 0.12, 0);
            }
            else if (targetButton.getName().equals("Right Button"))
            {
                Drive(0, -0.12, 0);
            }
            runtime.reset();
            do{}
            while(runtime.seconds() < 0.125);
            Stop();
        }

        /*MMTranslation currentLocation = null;
        boolean done = false;
        do {

            //visibleBeacon = getVisibleBeacon();
            if(visibleBeacon == null){
                Stop();
                logState("Unable to find beacon");
                return;
            }
            currentLocation = getCurrentLocation(visibleBeacon);

            if (currentLocation.getX() < targetButton.getMin())
            {
                Drive(0, -0.12, 0);
                //move to the right
            }
            else if (currentLocation.getX() > targetButton.getMax())
            {
                //move left
                Drive(0, 0.12, 0);
            }
            else
            {
                done = true;
            }

            //angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;

        } while(opModeIsActive() && !done && runtime.seconds() < 5);
        Stop();
        pauseBetweenSteps();

        /*double offset = 0;
        double moveSpeed = 0.15;
        if(targetButton.getName() == "Left Button") {
             offset = 1;
        } else {
            offset = -1;
        }

        Drive(0, moveSpeed * offset, 0);
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 0.3) {}

        Stop();*/
    }


    private void moveToBeacon(double x, double z, double angle) {
        double h=0,v=0,r=0;


        // slow movement into beacon as we get closer
        if (z < -650) {
            h = -0.35;
        } else if (z < -600) {
            h = -0.3;
        } else {
            h = -0.25;
        }

        //fix side to side movement

        if(Math.abs(x) > 100) {
            v = 0.15;
        } else if(Math.abs(x) > 50) {
            v = 0.1;
        } else if(Math.abs(x) < 25){
            v = 0.0;
        }

        if(x < 0) {
            v = 0-v;
        }


        if(Math.abs(angle) > 10) {
            r = 0.1;
        }
        if (Math.abs((angle)) > 5) {
            r = 0.05;
        } else if (Math.abs((angle)) < 2.1) {
            r = 0;
        }

        if(angle < 0) {
            r = 0-r;
        }

        if (opModeIsActive())
        {
            Drive(h, v, r);
        }


    }



    private VuforiaTrackableDefaultListener moveOffWall() {
        VuforiaTrackableDefaultListener visibleBeacon = null;

        runtime.reset();
        Drive(-0.9, -0.9, 0);
        do {
            //visibleBeacon = getVisibleBeacon();
            logState("[MOVE_FROM_START] moving from start [%f]", runtime.seconds());

        } while (opModeIsActive() && runtime.seconds() < 2);
        Stop();


        logState("[MOVE_FROM_START] move over in front of beacon");
        //pauseBetweenSteps();
        //move more
        runtime.reset();
        Drive(0, -0.9, 0);
        do {
        } while(opModeIsActive() && runtime.seconds() < 0.8);

        Stop();

        do {
            if (teamColor == TeamColor.RED)
            {
                visibleBeacon = getBeacon("gears"); //getVisibleBeacon();
            }
            else if (teamColor == TeamColor.BLUE)
            {
                visibleBeacon = getBeacon("wheels");
            }
        }while (opModeIsActive() && runtime.seconds() < 5 && visibleBeacon == null);

        //pauseBetweenSteps();

        /*if(visibleBeacon != null){
            //move back a bit?
            runtime.reset();
            Drive(0, 0.2, 0);
            while(opModeIsActive() && runtime.seconds() < 0.6) {

            }

            Stop();
        }*/

        Stop();

        pauseBetweenSteps();

        return visibleBeacon;
    }

    private void moveBackToSecondBeacon()
    {
        /*runtime.reset();
        Drive(0.4, 0, 0);
        do {
        } while(opModeIsActive() && runtime.seconds() < 1.1);*/

        runtime.reset();
        Drive(0, 0.9, 0);
        do {
        } while(opModeIsActive() && runtime.seconds() < 1.73);

        Stop();

        pauseBetweenSteps();
    }

    public void tempLog(String str)
    {
        robot.dashboard.displayPrintf(4, str);
    }

    private void runBeaconPressManuever (VuforiaTrackableDefaultListener visibleBeacon){

        ButtonRange targetButton = null;//ButtonRange.Unknown();


        boolean fixAngle1 = true;
        boolean centerCorrectButton = true;
        boolean moveToBeacon = true;
        boolean fixAngle2 = true;
        boolean fixAngle3 = true;
        boolean fixAngle4 = true;
        boolean getBeaconConfiguration = true;
        boolean moveToButton = false;
        boolean pressButton = true;


        robot.dashboard.displayText(15, "Fix angle");
        //fix angle
        if(fixAngle1) {
            fixAngles(visibleBeacon);
            //fixAngles(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }


        /*robot.dashboard.displayText(15, "Center on beacon");
        if(centerOnBeacon) {
            centerOnBeacon(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }*/

        robot.dashboard.displayText(15, "Get beacon configuration");

        //targetButton = null;

        Stop();
        waitFor(1);

        //get beacon configuration
        if(getBeaconConfiguration) {
            targetButton = getTargetButton(visibleBeacon);
            if(targetButton == null || targetButton == ButtonRange.Unknown()){
                robot.dashboard.displayText(10, "UNABLE TO FIND TEAM COLOR");
                return;
            }
            pauseBetweenSteps();
        }

        robot.dashboard.displayText(13, "Beacon Config: " + targetButton.getName());







       robot.dashboard.displayText(15, "Move towards beacon");
        //move towards beacon
        if(moveToBeacon) {
            moveToBeacon(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }

        if(fixAngle2) {
            fixAngles(visibleBeacon);

            //fixAngles(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }

        // Get in front of correct Button
        if (centerCorrectButton)
        {
            if (opModeIsActive())
            {
                if (targetButton.getName().equals("Left Button"))
                {
                    Drive(0, 0.12, 0);
                }
                else if (targetButton.getName().equals("Right Button"))
                {
                    Drive(0, -0.12, 0);
                }
                runtime.reset();
                do{}
                while(runtime.seconds() < 0.125);
                Stop();
            }

            check to make sure that you are centering on the beacon correctly, it seems like its not, check out the code doug had to progressively slow down

           /* MMTranslation currentLocation = null;
            boolean done = false;
            do {

                //visibleBeacon = getVisibleBeacon();
                if(visibleBeacon == null){
                    Stop();
                    logState("Unable to find beacon");
                    return;
                }
                currentLocation = getCurrentLocation(visibleBeacon);

                if (currentLocation.getX() < targetButton.getMin())
                {
                    Drive(0, -0.1, 0);
                    //move to the right
                }
                else if (currentLocation.getX() > targetButton.getMax())
                {
                    //move left
                    Drive(0, 0.1, 0);
                }
                else
                {
                    done = true;
                }

                //angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;

            } while(opModeIsActive() && !done && runtime.seconds() < 10);
            Stop();
            pauseBetweenSteps();*/
        }

        robot.dashboard.displayText(15, "Fix angle");
        //fix angle
        if(fixAngle3) {
            fixAngles(visibleBeacon);

            //fixAngles(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }




        if(targetButton.getName() == "Unknown") {
            logState("[MOVE TO CORRECT BUTTON] Unable to determine color, skipping");
        } else {

            logState("[MOVE TO CORRECT BUTTON] doing the do");
            robot.dashboard.displayText(15, "Move to correct button");
            //move to correct side
//            if (moveToButton) {
//                moveToCorrectButton(targetButton, visibleBeacon);
//                Stop();
//                pauseBetweenSteps();
//
//            }


            robot.dashboard.displayText(15, "Move in to press button");
            //move in to press button
            if (pressButton) {

                //move in
                runtime.reset();
                double moveSpeed = 0.4;
                Drive(0 - moveSpeed, 0.02, 0);
                do {
                    //idle();
                } while (opModeIsActive() && runtime.seconds() < 1.5);

                Stop();


                pauseBetweenSteps();


                //move out
                runtime.reset();
                Drive(0.9, 0, 0);
                do {
                } while (opModeIsActive() && runtime.seconds() < 0.9);
                Stop();

                pauseBetweenSteps();


                if(fixAngle4) {
                    fixAngles(visibleBeacon);

                    //fixAngles(visibleBeacon);
                    Stop();
                    pauseBetweenSteps();
                }
            }
        }
    }


    private void shootBalls() {

        double power = 0.4;
        while(power < 1 && opModeIsActive()){

            power += 0.005;
            power = Range.clip(power, 0, 1);
            robot.shooterMotor.setPower(power);

            if(power == 1) {
                break;
            }
            robot.dashboard.displayPrintf(2, "Waiting for shooter power: %2.5f", power);
            waitFor(0.02);
        }

        if (!opModeIsActive())
        {
            return;
        }

        waitFor(0.1);

        robot.dashboard.displayText(3, "Turning on elevator");
        robot.ElevatorLiftBalls();
        robot.dashboard.displayText(4, "Waiting for 0.5 seconds");
        waitFor(0.5);
        robot.dashboard.displayText(4, "Stopping elevator. let wheels spin back up");
        robot.ElevatorStop();
        robot.dashboard.displayText(4, "Waiting for 1 second");
        waitFor(1);

        robot.dashboard.displayText(4, "Turning on elevator");
        robot.ElevatorLiftBalls();
        robot.dashboard.displayText(4, "Waiting for 1 second");
        waitFor(1);


        robot.shooterMotor.setPower(0);
        robot.ElevatorStop();

        robot.StopAllMotors();

    }

    public void prepShooter () {
        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.shooterMotor.setMaxSpeed( (2200 * 28) / 60);

    }


    private void moveBackToShoot()
    {
        runtime.reset();
        Drive(0.9, 0, 0);
        do {
        } while(opModeIsActive() && runtime.seconds() < .4);

        Stop();
    }

    private void moveFwdToPark()
    {
        runtime.reset();
        Drive(0, 0.9, 0);
        do {
        } while(opModeIsActive() && runtime.seconds() < 0.4);
        Stop();
    }


    private void moveBackAndFixAngle( VuforiaTrackableDefaultListener visibleBeacon)
    {


        fixAngles(visibleBeacon);
        fixAngles(visibleBeacon);
    }

    private FtcAndroidAccel accelerometer;
    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        robot.dashboard.displayText(0, "*****WAAAAAIITT!!!!!  for it to say ready");

        accelerometer = new FtcAndroidAccel("Accelerometer", null);
        accelerometer.setEnabled(true);

        if (!OpenCVLoader.initDebug()) {
            logState("Unable to initialize opencv");
            pauseBetweenSteps();
            //Logger.d("Internal OpenCV library not found. Using OpenCV Manager for initialization");
            //OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, null);
        } else {
            //Logger.d("OpenCV library found inside package. Using it!");
            //mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        initVuforia();

        robot.dashboard.displayText(0, "Autonomous mode READY, waiting for start...");
        waitForStart();
        /************************************************************************************************************************/

        /*while (opModeIsActive())
        {
            TrcSensor.SensorData<Double> dataX = accelerometer.getXAcceleration();
            TrcSensor.SensorData<Double> dataY = accelerometer.getYAcceleration();
            TrcSensor.SensorData<Double> dataZ = accelerometer.getZAcceleration();
            robot.dashboard.displayText(5, String.valueOf(dataX.value));
            robot.dashboard.displayText(6, String.valueOf(dataY.value));
            robot.dashboard.displayText(7, String.valueOf(dataZ.value));
        }*/

        robot.dashboard.displayText(3, "Color: " + teamColor);


        if (opModeIsActive()) {
            runNearWithBothBeacons();
        }
    }


    private void runNearWithBothBeacons() throws InterruptedException {




        //VuforiaTrackableDefaultListener visibleBeacon = null;


        boolean moveOffWall = true;
        boolean runBeaconManuever = true;

        boolean press2ndbeacon = true;

        boolean shootBalls = false;
        boolean park = false;



        robot.dashboard.displayText(15, "Move off wall");
        VuforiaTrackableDefaultListener nearBeacon = null;
        if(moveOffWall) {

            nearBeacon = moveOffWall();      //returns the beacon that is nearest
            Stop();
        }

        if(runBeaconManuever && nearBeacon!= null) {
            runBeaconPressManuever(nearBeacon);

            //runtime.reset();


            fixAngles(nearBeacon);
            Stop();
            pauseBetweenSteps();

        }



        //move to 2nd beacon
        if(press2ndbeacon) {




            double firstToSecond = 1.6;

            if (teamColor == TeamColor.BLUE)
            {
                if (lastTarget.getName().equals("Right Button"))
                {
                    firstToSecond += 0.2;
                }
            }
            if (teamColor == TeamColor.RED)
            {
                if (lastTarget.getName().equals("Left Button"))
                {
                    firstToSecond += 0.2;
                }
            }


            runtime.reset();
            Drive(0, -0.9, 0);
            do {
            } while(opModeIsActive() && runtime.seconds() < firstToSecond);

            /*Drive(0.9, 0, 0);
            do {
            } while(opModeIsActive() && runtime.seconds() < 0.7);*/

            Stop();

            //waitFor(10); /*******Pause to move beacon over to other side manually******/

            //moveBackToSecondBeacon();

            //waitFor(1);

            // gears then tools

            VuforiaTrackableDefaultListener visibleBeacon = null;
            runtime.reset();
            do {
                if (teamColor == TeamColor.RED)
                {
                    visibleBeacon = getBeacon("tools");//getVisibleBeacon();
                }
                else if (teamColor == TeamColor.BLUE)
                {
                    visibleBeacon = getBeacon("legos");
                }

            }while(opModeIsActive() && runtime.seconds() < 5 && visibleBeacon == null);

            //waitFor(1);


            if (visibleBeacon != null)
            {
               // moveBackAndFixAngle(visibleBeacon);

                runBeaconPressManuever(visibleBeacon);
            }


            pauseBetweenSteps();

        }


        if (shootBalls)
        {

            runtime.reset();
            robot.shooterMotor.setPower(-1);
            do{}
            while (opModeIsActive() && runtime.seconds() < 1);

            Stop();

            pauseBetweenSteps();
        }


        if (park)
        {
            runtime.reset();
            Drive(-0.9, 0, 0);
            do {
            } while(opModeIsActive() && runtime.seconds() < 2);
            Stop();
        }




    }


    //all drive 'v' values configured as red. reverse for blue
    private void Drive(double h, double v, double r) {

        if(teamColor == TeamColor.BLUE){
            v *= -1;
        }

        mechDrive.Drive(h, v, r, false);
    }
    private void Stop() {
        mechDrive.Stop();
    }

    private void logState(String msg, Object... args){
        robot.dashboard.displayPrintf(11, msg, args);
    }



    private void pauseBetweenSteps(){
        //logPath("pausing waiting 2 seconds");
        waitFor(0.25);
    }


    private void waitFor(double seconds){
        robot.waitForTick((long)(seconds * 1000));
    }


    //this assumes the horizontal axis is the y-axis since the phone is vertical
    //robot angle is relative to "parallel with the beacon wall"
    public MMTranslation navOffWall(VectorF trans, double robotAngle, VectorF offWall) {
        return new MMTranslation( new VectorF(
                (float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1),
                (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))))
        );
    }

    public MMTranslation anglesFromTarget(VuforiaTrackableDefaultListener image) {
        try {

            if (image == null)
            {
                return null;
            }

            float [] data = image.getRawPose().getData();

            float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
            double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
            double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
            double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
            return new MMTranslation( new VectorF((float)thetaX, (float)thetaY, (float)thetaZ)) ;
        }
        catch (NullPointerException e)
        {
            return null;
        }

    }


    class MMTranslation{

        private double x;
        private double y;
        private double z;

        public double getX(){
            return  x;
        }
        public double getY(){
            return y;
        }
        public double getZ() {
            return  z;
        }

        public double getAngle() {

            //double angle = Math.toDegrees(Math.atan2(z, x)) + 90;


            double angle = Math.toDegrees(Math.atan2(x, -z));
            return angle;

        }

        public MMTranslation(VectorF translation){
            if(translation == null){
                return;
            }
            x = translation.get(0);
            y = translation.get(1);      //not really used
            z = translation.get(2);
        }





    }




    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        OpenGLMatrix pose = beacon.getRawPose();

        if (pose != null && img != null && img.getPixels() != null) {

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

            rawPose.setData(poseData);

            //calculating pixel coordinates of beacon corners
            float[][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 92, 0)).getData(); //lower right of beacon
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 92, 0)).getData(); //lower left of beacon

            //getting camera image...
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            //turning the corner pixel coordinates into a proper bounding box
            Mat crop = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);
            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));


            //make sure our bounding box doesn't go outside of the image
            //OpenCV doesn't like that...
            x = Math.max(x, 0);
            y = Math.max(y, 0);
            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            //cropping bounding box out of camera image
            final Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            Bitmap pic = OCVUtils.matToBitmap(cropped);
            //filtering out non-beacon-blue colours in HSV colour space
            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);




            /*try
            {
                FileOutputStream out = new FileOutputStream(new File("/storage/emulated/0/", "poop.txt"));
                out.write((new String("ppoooop")).getBytes());
                out.close();
            } catch (FileNotFoundException e){}
            catch (IOException e){}
*/

            /*try
            {
                FileOutputStream fos = new FileOutputStream(new File("/storage/emulated/0/", "cropped.png"));

                //bm.compress(Bitmap.CompressFormat.PNG, 90, fos);
                if (pic.compress(Bitmap.CompressFormat.PNG, 100, fos))
                {
                }
                else
                {

                }
                fos.close();
            }catch (IOException e)
            {}

            try
            {
                FileOutputStream fos = new FileOutputStream(new File("/storage/emulated/0/", "non.png"));

                //bm.compress(Bitmap.CompressFormat.PNG, 90, fos);
                if (bm.compress(Bitmap.CompressFormat.PNG, 100, fos))
                {
                }
                else
                {
                    tempLog("didgfeds");
                }
                fos.close();
            }catch (IOException e)
            {}
*/










            //get filtered mask
            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
            //Otherwise, it's turned to black
            Mat mask = new Mat();

            Core.inRange(cropped, BEACON_BLUE_LOW, BEACON_BLUE_HIGH, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            //calculating centroid of the resulting binary mask via image moments
            Log.i("CentroidX", "" + ((mmnts.get_m10() / mmnts.get_m00())));
            Log.i("CentroidY", "" + ((mmnts.get_m01() / mmnts.get_m00())));

            //checking if blue either takes up the majority of the image (which means the beacon is all blue)
            //or if there's barely any blue in the image (which means the beacon is all red or off)
//            if (mmnts.get_m00() / mask.total() > 0.8) {
//                return VortexUtils.BEACON_ALL_BLUE;
//            } else if (mmnts.get_m00() / mask.total() < 0.1) {
//                return VortexUtils.BEACON_NO_BLUE;
//            }//elseif

            //Note: for some reason, we end up with a image that is rotated 90 degrees
            //if centroid is in the bottom half of the image, the blue beacon is on the left
            //if the centroid is in the top half, the blue beacon is on the right
            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2) {
                return VortexUtils.BEACON_RED_BLUE;
            } else {
                return VortexUtils.BEACON_BLUE_RED;
            }//else
        }//if

        return VortexUtils.NOT_VISIBLE;
    }//getBeaconConfig


}



