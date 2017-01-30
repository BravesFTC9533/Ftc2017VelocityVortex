package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.widget.Button;

import com.qualcomm.ftcrobotcontroller.R;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Image;
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
                color = VortexUtils.getBeaconConfig(img, listener, vuforia.getCameraCalibration());
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
        } while(opModeIsActive() && currentLocation.getZ() < -300 && runtime.seconds() < 10);
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

    private void moveToCorrectButton(ButtonRange targetButton) {


        double offset = 0;
        double moveSpeed = 0.25;
        if(targetButton.getName() == "Left Button") {
             offset = 1;
        } else {
            offset = -1;
        }

        Drive(0, moveSpeed * offset, 0);
        runtime.reset();

        while(opModeIsActive() && runtime.seconds() < 0.3) {}

        Stop();
    }


    private void moveToBeacon(double x, double z, double angle) {
        double h=0,v=0,r=0;


        // slow movement into beacon as we get closer
        if (z < -650) {
            h = -0.4;
        } else if (z < -600) {
            h = -0.35;
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

        } while (opModeIsActive() && runtime.seconds() < 2.9);
        Stop();


        logState("[MOVE_FROM_START] move over in front of beacon");
        //pauseBetweenSteps();
        //move more
        runtime.reset();
        Drive(0, -0.9, 0);
        do {
        } while(opModeIsActive() && runtime.seconds() < 0.6);

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

    private void runBeaconPressManuever (VuforiaTrackableDefaultListener visibleBeacon){

        ButtonRange targetButton = null;//ButtonRange.Unknown();


        boolean fixAngle1 = true;
        boolean centerOnBeacon = false;
        boolean moveToBeacon = true;
        boolean fixAngle2 = true;
        boolean getBeaconConfiguration = true;
        boolean moveToButton = true;
        boolean pressButton = true;


        robot.dashboard.displayText(15, "Fix angle");
        //fix angle
        if(fixAngle1) {
            fixAngles(visibleBeacon);
            //fixAngles(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }


        robot.dashboard.displayText(15, "Center on beacon");
        if(centerOnBeacon) {
            centerOnBeacon(visibleBeacon);
            Stop();
            pauseBetweenSteps();
        }

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

        robot.dashboard.displayText(15, "Fix angle");
        //fix angle
        if(fixAngle2) {
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
            if (moveToButton) {
                moveToCorrectButton(targetButton);
                Stop();
                pauseBetweenSteps();

            }


            robot.dashboard.displayText(15, "Move in to press button");
            //move in to press button
            if (pressButton) {

                //move in
                runtime.reset();
                double moveSpeed = 0.4;
                Drive(0 - moveSpeed, 0, 0);
                do {
                    idle();
                } while (opModeIsActive() && runtime.seconds() < 0.7);

                Stop();
                pauseBetweenSteps();



                /*int mycolor = getBeaconColor(visibleBeacon);

                if (mycolor == -1)
                {
                    // move on
                }
                else if (mycolor == VortexUtils.BEACON_ALL_BLUE)
                {
                    if (teamColor == TeamColor.BLUE)
                    {
                        // do nothing
                    }
                    else if (teamColor == TeamColor.RED)
                    {
                        runBeaconPressManuever(visibleBeacon);
                    }
                }
                else        // all red
                {
                    if (teamColor == TeamColor.BLUE)
                    {
                        runBeaconPressManuever(visibleBeacon);
                    }
                    else if (teamColor == TeamColor.RED)
                    {
                        //do nothing
                    }
                }*/




                //move out
                /*runtime.reset();
                Drive(0.6, 0, 0);
                do {
                    idle();
                } while (opModeIsActive() && runtime.seconds() < 0.7);*/
                Stop();

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

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        robot.dashboard.displayText(0, "WAAAAAIITT!!!!!  for it to say ready");

        //teamColor = TeamColor.RED;



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

        robot.dashboard.displayText(0, "Autonomous mode ready, waiting for start...");
        waitForStart();

//        runtime.reset();
//        while(opModeIsActive() && runtime.seconds() < 6) {
//
//        }
       // waitFor(3);

        robot.dashboard.displayText(3, "Color: " + teamColor);

        if (opModeIsActive())
        {
            if (proximity == Proximity.FAR)
            {
                if (numBeacons == 0)
                {

                }
                else if (numBeacons == 1)
                {

                }
                else if (numBeacons == 2)
                {

                }
            }
            if (proximity == Proximity.NEAR)
            {
                if (numBeacons == 0)
                {
                    runNearWithoutBeacons();
                }
                else if (numBeacons == 1)
                {

                }
                else if (numBeacons == 2)
                {
                    runNearWithBothBeacons();
                }
            }
        }


    }

    /**
     * Goes for:
     *
     * Shoots
     * Push Capball
     * park in center
     *
     */
    private void runNearWithoutBeacons()
    {

    }

    /**
     * Goes for:
     *
     * Tools Beacon
     * Gears Beacon
     * Turn and Shoot
     *
     * (Park on ramp?)
     */



    private void runNearWithBothBeacons() throws InterruptedException {




        //VuforiaTrackableDefaultListener visibleBeacon = null;


        boolean moveOffWall = true;
        boolean runBeaconManuever = true;

        boolean press2ndbeacon = true;

        boolean shootBalls = true;
        boolean park = false;



        robot.dashboard.displayText(15, "Move off wall");
        VuforiaTrackableDefaultListener farBeacon = null;
        if(moveOffWall) {

            farBeacon = moveOffWall();      //returns the beacon that is farthest
            Stop();
        }

        if(runBeaconManuever && farBeacon!= null) {
            runBeaconPressManuever(farBeacon);

            //runtime.reset();

            //back out from wall
            Drive(0.9, 0, 0);
            do {
            } while(opModeIsActive() && runtime.seconds() < .8);
            Stop();


            double firstToSecond = 1.5;

            if (teamColor == TeamColor.BLUE)
            {
                if (lastTarget.getName().equals("Right Button"))
                {
                    firstToSecond += 0.4;
                }
            }
            if (teamColor == TeamColor.RED)
            {
                if (lastTarget.getName().equals("Left Button"))
                {
                    firstToSecond += 0.4;
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
        }



        //move to 2nd beacon
        if(press2ndbeacon) {

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


        if(shootBalls) {

            moveBackToShoot();

            if (teamColor == TeamColor.BLUE)
            {
                if (lastTarget.getName().equals("Left Button")) {
                    turn(-140);

                    prepShooter();

                    shootBalls();

                    //turn(100);
                }
                else if (lastTarget.getName().equals("Right Button"))
                {
                    turn(-50);

                    prepShooter();

                    shootBalls();

                    //turn(95);
                }
            }
            else if (teamColor == TeamColor.RED)
            {
                if (lastTarget.getName().equals("Left Button")) {
                    turn(-50);

                    prepShooter();

                    shootBalls();

                    //turn(100);
                }
                else if (lastTarget.getName().equals("Right Button"))
                {
                    turn(-50);

                    prepShooter();

                    shootBalls();

                    //turn(95);
                }
            }




            moveFwdToPark();
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




}



