package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.widget.Button;

import com.qualcomm.ftcrobotcontroller.R;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
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
import org.firstinspires.ftc.teamcode.Util.VortexUtils;
import org.opencv.android.OpenCVLoader;

import static org.firstinspires.ftc.teamcode.Util.VortexUtils.getImageFromFrame;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vuforia-Ryan", group = "vuf")
//@Autonomous(name="Concept: Vuforia Navigation", group =sensorManager = (SensorManager) Activity.getSystemService(SENSOR_SERVICE);"Concept")
public class VuforiaOp extends MMOpMode_Linear{ //extends MMOpMode_Linear{



    enum TeamColor {
        BLUE,
        RED
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

    TeamColor teamColor = TeamColor.RED;

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AeWceoD/////AAAAGWvk7AQGLUiTsyU4mSW7gfldjSCDQHX76lt9iPO5D8zaboG428rdS9WN0+AFpAlc/g4McLRAQIb5+ijFCPJJkLc+ynXYdhljdI2k9R4KL8t3MYk/tbmQ75st9VI7//2vNkp0JHV6oy4HXltxVFcEbtBYeTBJ9CFbMW+0cMNhLBPwHV7RYeNPZRgxf27J0oO8VoHOlj70OYdNYos5wvDM+ZbfWrOad/cpo4qbAw5iB95T5I9D2/KRf1HQHygtDl8/OtDFlOfqK6v2PTvnEbNnW1aW3vPglGXknX+rm0k8b0S7GFJkgl7SLq/HFNl0VEIVJGVQe9wt9PB6bJuxOMMxN4asy4rW5PRRBqasSM7OLl4W";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

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
            color = VortexUtils.waitForBeaconConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), listener, vuforia.getCameraCalibration(), 5000);
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
            pauseBetweenSteps();
            pauseBetweenSteps();
            pauseBetweenSteps();

        }

        return color;
    }




    private ButtonRange getTargetButton(){

        VuforiaTrackableDefaultListener visibleBeacon = getVisibleBeacon();
        ButtonRange targetButton = null;
        int mycolor = getBeaconColor(visibleBeacon);

        if(mycolor == -1) {
            logState("[GET_BEACON_COLOR] Unable to determine color");

            return ButtonRange.RightButton();

        } else {
            logState("[GET_BEACON_COLOR] Got a beacon color");
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
                logState("[GET_BEACON_COLOR] Unable to determine color");

                return ButtonRange.RightButton();
            }
            return  targetButton;
        }


    }

    private void fixAngles(){
        logState("[SQUARE UP TO WALL]");
        VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation angles = null;
        double angleToWall = 0;


//        visibleBeacon = getVisibleBeacon();
//        if(visibleBeacon == null){
//            //uh oh
//            logState("Unable to locate beacon");
//            idle();
//
//        }
//        angles = anglesFromTarget(visibleBeacon);
//        angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
//        logState("[SQUARE UP TO WALL] Angle: %f", angleToWall - 90);
//


        //fix angle
        runtime.reset();
        do {
            visibleBeacon = getVisibleBeacon();
            if(visibleBeacon == null){
                //uh oh
                logState("Unable to locate beacon");
                idle();
                continue;
            }
            angles = anglesFromTarget(visibleBeacon);
            angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
            logState("[SQUARE UP TO WALL] Angle: %f", angleToWall - 90);
            fixAngle(angleToWall - 90);

        } while(opModeIsActive() && Math.abs(angleToWall - 90) > 3 && runtime.seconds() < 2);
    }


    private void moveToBeacon() {
        logState("[MOVE_TO_BEACON] Move closer to beacon");

        VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation angles;
        MMTranslation currentLocation = null;
        double angleToWall;
        //currentLocation = getCurrentLocation(visibleBeacon);
        runtime.reset();
        do {
            visibleBeacon = getVisibleBeacon();
            if(visibleBeacon == null){
                mechDrive.Stop();
                logState("Unable to find beacon");
                return;
            }
            currentLocation = getCurrentLocation(visibleBeacon);
            angles = anglesFromTarget(visibleBeacon);
            angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
            moveToBeacon(currentLocation.getX(), currentLocation.getZ(), angleToWall - 90);
        } while(opModeIsActive() && currentLocation.getZ() < -550 && runtime.seconds() < 10);
    }


    private void moveToCorrectButton(ButtonRange targetButton) {


        double offset = 0;
        double moveSpeed = 0.125;
        if(targetButton.getName() == "Left Button") {
             offset = 1;
        } else {
            offset = -1;
        }

        mechDrive.Drive(0, moveSpeed * offset, 0, false);
        runtime.reset();

        do {

            idle();
        } while(opModeIsActive() && runtime.seconds() < 1);
        mechDrive.Stop();
    }
    private void moveToCorrectButton_old(ButtonRange targetButton) {

        VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation angles;
        MMTranslation currentLocation = null;
        double angleToWall;
        double moveSpeed = 0.125;


        runtime.reset();
        int offset = 0;
        if(targetButton.getName() == "Left Button") {
            offset = -25;
        } else {
            offset = 25;
        }

        MMTranslation nav = null;
        boolean firstTime = true;
        do {
            visibleBeacon = getVisibleBeacon();
            if(visibleBeacon == null){
                mechDrive.Stop();
                idle();
                continue;
            }

            currentLocation = getCurrentLocation(visibleBeacon);
            angles = anglesFromTarget(visibleBeacon);
            nav = navOffWall(visibleBeacon.getPose().getTranslation(), Math.toDegrees(angles.getX()) - 90, new VectorF(offset, 0, 150));
            logState("[MOVE TO BUTTON] Nav: x[%f], z[%f]", nav.getX(), nav.getZ());


            robot.dashboard.displayPrintf(2, "Current X:" + currentLocation.getX());
            robot.dashboard.displayPrintf(3, "Current Z:" + currentLocation.getZ());

            robot.dashboard.displayPrintf(15, "");

            double currentX = nav.getX();
            if(Math.abs(currentX) > 20) {
                moveSpeed = 0.125;
            } else if(Math.abs(currentX) > 10) {
                moveSpeed = 0.100;
            } else {
                moveSpeed = 0;
            }

            if(currentX < 0) {
                moveSpeed *= -1;
            }

            robot.dashboard.displayPrintf(14, "Move: h[0] v[%f] r[0]", moveSpeed);

            if(firstTime) {
                pauseBetweenSteps();
                pauseBetweenSteps();
                firstTime = false;
            }

            mechDrive.Drive(0, moveSpeed, 0, false);

        } while(opModeIsActive() && runtime.seconds() < 30 && Math.abs(nav.getX()) > 10);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

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


        boolean atBeacon = false;

        robot.dashboard.displayText(0, "Autonomous mode ready. Press start to begin..");
        waitForStart();

        runAuto();


//        ButtonRange targetButton = ButtonRange.LeftButton();
//
//        States state = States.MOVE_FROM_START;
//
//
//        VuforiaTrackableDefaultListener visibleBeacon = null;
//        MMTranslation currentLocation = null;
//
//        boolean move = true;
//
//        runtime.reset();
//
//        Global.opModeActive = opModeIsActive();
//
//        while (opModeIsActive()) {
//
//
//            MMTranslation angles = null;
//            MMTranslation nav = null;
//            double angleToWall = 0;
//
//
//            robot.dashboard.displayPrintf(10, "Compass Says X: " + Global.compass);
//
//
//            visibleBeacon = getVisibleBeacon();
//
//
//
//            if (visibleBeacon != null) {
//                currentLocation = getCurrentLocation(visibleBeacon);
//                if(currentLocation != null) {
//                    angles = anglesFromTarget(visibleBeacon);
//                    if (visibleBeacon.getPose() != null) {
//                        nav = navOffWall(visibleBeacon.getPose().getTranslation(), Math.toDegrees(angles.getX()) - 90, new VectorF(500, 0, 0));
//
//                        angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
//                    }
//                }
//
//            }
//
//            if (currentLocation == null) {
//                //state = States.FIND_BEACON;
//            } else {
//                //robot.dashboard.displayPrintf(0, "****** " + visibleBeacon.);
//                robot.dashboard.displayPrintf(1, "Turn " + currentLocation.getAngle() + " degrees");
//                robot.dashboard.displayPrintf(2, "X:" + currentLocation.getX());
//                robot.dashboard.displayPrintf(3, "Z:" + currentLocation.getZ());
//                if(nav != null) {
//                    robot.dashboard.displayPrintf(4, "Nav: x[%f], z[%f]", nav.getX(), nav.getZ());
//                }
//                if(angles!=null) {
//                    robot.dashboard.displayPrintf(5, "Angle: x[%f], z[%f]", (Math.toDegrees(angles.getX()) + 270) % 360, Math.toDegrees(angles.getZ()));
//                }
//            }
//
//
//
//            robot.dashboard.displayPrintf(9, "Current Button Target: " + targetButton.getName());
//
//
//
//            switch (state) {
//
//                case MOVE_FROM_START:
//                    if(runtime.seconds() < 4) {
//                        logState("[MOVE_FROM_START] moving from start [%f]", runtime.seconds());
//                        mechDrive.Drive(-0.2, -0.4, 0, true);
//                    } else {
//                        mechDrive.Stop();
//
//                        state = States.FIND_BEACON;
//                        pauseBetweenSteps();
//                    }
//                case FIND_BEACON:
//                    logState("[FIND_BEACON] Finding beacon");
//                    // get visible beacon
//
//                    if(visibleBeacon != null){
//                        state = States.GET_BEACON_LOCATION;
//
//                        //pauseBetweenSteps();
//
//                    } else {
//                        state = States.FIND_BEACON;
//                    }
//                    break;
//
//                case GET_BEACON_LOCATION:
//                    logState("[GET_BEACON_LOCATION] Getting beacon location");
//
//                    if(currentLocation!=null)
//                    {
//                        state = States.GET_BEACON_COLOR;
//                        //pauseBetweenSteps();
//                    } else {
//                        state = States.FIND_BEACON;
//                    }
//                    break;
//
//                case GET_BEACON_COLOR:
//                    logState("[GET_BEACON_COLOR] Getting beacon color");
//
//                    int mycolor = getBeaconColor(visibleBeacon);
//
//
//                    if(mycolor == -1) {
//                        logState("[GET_BEACON_COLOR] Unable to determine color");
//                        pauseBetweenSteps();
//                    } else {
//                        logState("[GET_BEACON_COLOR] Got a beacon color");
//                        if (mycolor == VortexUtils.BEACON_BLUE_RED) {
//
//                            if (teamColor == TeamColor.BLUE) {
//                                targetButton = ButtonRange.LeftButton();
//                            } else {
//                                targetButton = ButtonRange.RightButton();
//                            }
//                        } else if (mycolor == VortexUtils.BEACON_RED_BLUE) {
//                            if (teamColor == TeamColor.BLUE) {
//                                targetButton = ButtonRange.RightButton();
//                            } else {
//                                targetButton = ButtonRange.LeftButton();
//                            }
//                        } else {
//
//                        }
//                        state = States.MOVE_TO_BEACON;
//                    }
//                    break;
//                case MOVE_TO_BEACON: //get closer to beacon
//                    logState("[MOVE_TO_BEACON] Move closer to beacon");
//                    //currentLocation = getCurrentLocation(visibleBeacon);
//
//                    if(currentLocation.getZ() < -300) {
//                        //robot.dashboard.displayPrintf(11, "Moving to beacon");
//                        moveToBeacon(currentLocation.getX(), currentLocation.getZ(), angleToWall - 90);
//
//                    } else {
//                        mechDrive.Stop();
//                        state = States.FIX_ANGLE;
//                        pauseBetweenSteps();
//                        runtime.reset();
//                    }
//                    break;
//                case FIX_ANGLE:
//                    logState("[FIX_ANGLE] fixing angle");
//
//
//                    if(Math.abs(angleToWall - 90) > 5 && runtime.seconds() < 2) {
//                        fixAngle(angleToWall -90);
//                    } else {
//                        mechDrive.Stop();
//                        state = States.MOVE_TO_CORRECT_BUTTON;
//                        pauseBetweenSteps();
//                    }
//
//
//                    break;
//                case MOVE_TO_CORRECT_BUTTON: //move to correct button
//                    logState("[MOVE_TO_CORRECT_BUTTON] move left or right to button");
//                    //currentLocation = getCurrentLocation(visibleBeacon);
//                    if(!targetButton.inRange(currentLocation.getX())){
//                        moveToX(targetButton, currentLocation.getX(), currentLocation.getX());
//                    } else {
//                        mechDrive.Stop();
//                        state = States.PRE_MOVE_IN;
//                        pauseBetweenSteps();
//                    }
//                    break;
//
//                case PRE_MOVE_IN: //press button begin
//                    logState("[PRE_MOVE_IN] begin button press");
//                    runtime.reset();
//                    state = States.PRESS_BUTTON;
//                    //pauseBetweenSteps();
//                    break;
//                case PRESS_BUTTON: //move into button
//                    logState("[PRESS_BUTTON] move into button");
//                    if(runtime.seconds() < 1.2) {
//                        mechDrive.Drive(-0.2, 0, 0, false);
//                    } else {
//                        state = States.PRE_MOVE_OUT;
//
//                        mechDrive.Stop();
//                        pauseBetweenSteps();
//                    }
//                    break;
//                case PRE_MOVE_OUT: //move away from button start
//                    logState("[PRE_MOVE_OUT] begin stop button press");
//                    runtime.reset();
//                    state = States.BACK_OFF_BUTTON;
//                    break;
//                case BACK_OFF_BUTTON: //move away from button
//                    logState("[BACK_OFF_BUTTON] move away from button");
//                    if(runtime.seconds() < 1){
//                        logState("[7.1] move backwards");
//                        mechDrive.Drive(0.2, 0, 0, false);
//                    } else {
//                        logState("[7.2] stopping");
//                        mechDrive.Stop();
//                        pauseBetweenSteps();
//                        state = States.DONE;
//                    }
//                    break;
//                case DONE:
//                    logState("[8] done");
//                    //do nothing
//                    break;
//
//            }
//
//
//            telemetry.update();
//
//            waitFor(0.02);
//        }
    }

    private void runAuto() throws InterruptedException {


        ButtonRange targetButton = ButtonRange.RightButton();
        VuforiaTrackableDefaultListener visibleBeacon = null;


        boolean moveOffWall = false;
        boolean findBeacon = false;
        boolean fixAngle1 = false;

        boolean moveToBeacon = true;
        boolean fixAngle2 = true;
        boolean getBeaconConfiguration = true;
        boolean moveToButton = true;

        boolean pressButton = true;


        robot.dashboard.displayText(15, "Move off wall");
        // move off wall
        if(moveOffWall) {
            runtime.reset();
            mechDrive.Drive(-0.25, -0.4, 0, true);
            do {
                visibleBeacon = getVisibleBeacon();
                logState("[MOVE_FROM_START] moving from start [%f]", runtime.seconds());
                idle();
            }while (opModeIsActive() && runtime.seconds() < 3 && visibleBeacon == null);

            mechDrive.Stop();
            pauseBetweenSteps();

        }

        robot.dashboard.displayText(15, "Find Beacon");
        //find beacon
        if(findBeacon) {
            runtime.reset();
            do {
                visibleBeacon = getVisibleBeacon();
                if (visibleBeacon == null) {
                    logState("[FIND BEACON] Unable to find beacon");
                } else {
                    logState("[FIND BEACON] Found beacon");
                    break;
                }
                idle();

            } while(opModeIsActive() && runtime.seconds() < 4);

        }
        pauseBetweenSteps();

        robot.dashboard.displayText(15, "Fix angle");
        //fix angle
        if(fixAngle1) {
            fixAngles();
            mechDrive.Stop();
            pauseBetweenSteps();
        }




        robot.dashboard.displayText(15, "Move towards beacon");
        //move towards beacon
        if(moveToBeacon) {
            moveToBeacon();
            mechDrive.Stop();
            pauseBetweenSteps();
        }

        robot.dashboard.displayText(15, "Fix angle");
        //fix angle
        if(fixAngle2) {
            fixAngles();
            mechDrive.Stop();
            pauseBetweenSteps();
        }

        robot.dashboard.displayText(15, "Get beacon configuration");
        //get beacon configuration
        if(getBeaconConfiguration) {
            targetButton = getTargetButton();
            if(targetButton == null){
                robot.dashboard.displayText(10, "UNABLE TO FIND TEAM COLOR");
            }
            pauseBetweenSteps();
        }

        robot.dashboard.displayText(14, "Beacon Config: " + targetButton.getName());



        robot.dashboard.displayText(15, "Move to correct button");
        //move to correct side
        if(moveToButton) {
            moveToCorrectButton(targetButton);
            mechDrive.Stop();
            pauseBetweenSteps();
            pauseBetweenSteps();
        }


        robot.dashboard.displayText(15, "Move in to press button");
        //move in to press button
        if(pressButton){

            //move in
            runtime.reset();
            double moveSpeed = 0.2;
            mechDrive.Drive(0-moveSpeed, 0, 0, false);
            do {
                idle();
            } while(opModeIsActive() && runtime.seconds() < 2) ;
            mechDrive.Stop();
            pauseBetweenSteps();
            pauseBetweenSteps();

            //move out
            runtime.reset();
            mechDrive.Drive(moveSpeed, 0, 0, false);
            do {
                idle();
            } while(opModeIsActive() && runtime.seconds() < 1) ;
            mechDrive.Stop();

        }

    }



    private void logState(String msg, Object... args){
        robot.dashboard.displayPrintf(11, msg, args);
    }


    private void moveToX(ButtonRange b, double x, double z) {
        double h = 0, v = 0, r = 0;

        if (x > b.getMax()) {
            v = 0.12;   //move left
        } else if (x < b.getMin()) {
            v = -0.12;     //move right
        } else {
            v = 0;
        }
        mechDrive.Drive(h, v, r, false);
        waitFor(0.2);
        mechDrive.Stop();



    }


    private void fixAngle(double angle){
        double r = 0;
        if(Math.abs(angle) > 10) {
            r = 0.15;
        } else if(Math.abs((angle)) > 5) {
            r = 0.115;
        } else if(Math.abs((angle)) < 2.1) {
            r = 0;
        }


        if(angle < 0) {
            r = 0-r;
        }

        mechDrive.Drive(0, 0, r, false);
    }

    private void moveToBeacon(double x, double z, double angle) {
        double h=0,v=0,r=0;


        // slow movement into beacon as we get closer
        if (z < -650) {
            h = -0.3;
        } else if (z < -600) {
            h = -0.2;
        } else {
            h = -0.16;
        }

        //fix side to side movement
        if(Math.abs(x) > 70) {
            v = 0.15;
            if(x < 0) {
                v = 0-v;
            }
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

        mechDrive.Drive(h, v, r, false);






    }

    private void goStraight(String step, double time, boolean forward){
        //robot.DriveRobot(FORWARD_SPEED, FORWARD_SPEED, telemetry);

        robot.dashboard.displayPrintf(0, "%s: Drive straight for %s second(s)", step, time);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
            robot.dashboard.displayPrintf(1, "Straight: %2.5f S Elapsed: %s", runtime.seconds(), robot.leftMotor.getCurrentPosition());

            double speed = 0.15;
            if(!forward){
                speed *= -1;
            }
            //robot.DriveRobot(FORWARD_SPEED, FORWARD_SPEED);
            mechDrive.Drive(0, speed, 0);
            robot.waitForTick(40);
        }

        mechDrive.Stop();
        pauseBetweenSteps();
    }


    private void pauseBetweenSteps(){
        //logPath("pausing waiting 2 seconds");
        waitFor(1);
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
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new MMTranslation( new VectorF((float)thetaX, (float)thetaY, (float)thetaZ)) ;
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




}



