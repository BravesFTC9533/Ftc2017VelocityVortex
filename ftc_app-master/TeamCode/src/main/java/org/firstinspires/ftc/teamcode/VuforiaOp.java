package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

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

import static org.firstinspires.ftc.teamcode.Util.VortexUtils.getImageFromFrame;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vuforia-Ryan", group = "vuf")
//@Autonomous(name="Concept: Vuforia Navigation", group =sensorManager = (SensorManager) Activity.getSystemService(SENSOR_SERVICE);"Concept")
public class VuforiaOp extends MMOpMode_Linear{ //extends MMOpMode_Linear{

    public static final String TAG = "Vuforia Sample";

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();

    private final int RED = 0, BLUE = 1;
    private String leftBeacon, rightBeacon;
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

    public int getBeaconColor(VuforiaTrackableDefaultListener listener)
    {
        int color = -1;

        try {
            color = VortexUtils.waitForBeaconConfig(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565), listener, vuforia.getCameraCalibration(), 5000);
        } catch (InterruptedException e){}

        if (color == VortexUtils.BEACON_BLUE_RED)
        {
            robot.dashboard.displayPrintf(15, "Blue on left, Red on right");
        }
        else if (color == VortexUtils.BEACON_RED_BLUE)
        {
            robot.dashboard.displayPrintf(15, "Red on left, Blue on right");
        }


        return color;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        initVuforia();


        boolean atBeacon = false;

        waitForStart();

        ButtonRange targetButton = ButtonRange.LeftButton();

        States state = States.FIND_BEACON;


        VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation currentLocation = null;

        boolean move = true;

        runtime.reset();

        Global.opModeActive = opModeIsActive();

        while (opModeIsActive()) {


            robot.dashboard.displayPrintf(10, "Compass Says X: " + Global.compass);


            visibleBeacon = getVisibleBeacon();

            if (visibleBeacon != null) {
                currentLocation = getCurrentLocation(visibleBeacon);
            }

            if (currentLocation == null) {
                state = States.FIND_BEACON;
            } else {
                //robot.dashboard.displayPrintf(0, "****** " + visibleBeacon.);
                robot.dashboard.displayPrintf(1, "Turn " + currentLocation.getAngle() + " degrees");
                robot.dashboard.displayPrintf(2, "X:" + currentLocation.getX());
                robot.dashboard.displayPrintf(3, "Z:" + currentLocation.getZ());
            }





            switch (state){

//                case MOVE_FROM_START:
//                    if(runtime.seconds() < 4) {
//                        logState("[MOVE_FROM_START] moving from start");
//                        mechDrive.Drive(-0.1, -0.4, 0, true);
//                    } else {
//                        mechDrive.Stop();
//
//                        state = States.FIND_BEACON;
//                        pauseBetweenSteps();
//                    }
                case FIND_BEACON:
                    logState("[FIND_BEACON] Finding beacon");
                    // get visible beacon

                    if(visibleBeacon != null){
                        state = States.GET_BEACON_LOCATION;

                        //pauseBetweenSteps();

                    } else {
                        state = States.FIND_BEACON;
                    }
                    break;

                case GET_BEACON_LOCATION:
                    logState("[GET_BEACON_LOCATION] Getting beacon location");

                    if(currentLocation!=null)
                    {
                        state = States.GET_BEACON_COLOR;
                        //pauseBetweenSteps();
                    } else {
                        state = States.FIND_BEACON;
                    }
                    break;

                case GET_BEACON_COLOR:
                    logState("[GET_BEACON_COLOR] Getting beacon color");

                    int mycolor = getBeaconColor(visibleBeacon);


                    if(mycolor == -1) {
                        logState("[GET_BEACON_COLOR] Unable to determine color");
                        pauseBetweenSteps();
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

                        }
                        state = States.MOVE_TO_BEACON;
                    }
                    break;
                case MOVE_TO_BEACON: //get closer to beacon
                    logState("[MOVE_TO_BEACON] Move closer to beacon");
                    //currentLocation = getCurrentLocation(visibleBeacon);
                    if(currentLocation.getZ() < -200) {
                        //robot.dashboard.displayPrintf(11, "Moving to beacon");
                        moveToBeacon(currentLocation.getX(), currentLocation.getZ(), currentLocation.getAngle());

                    } else {
                        mechDrive.Stop();
                        state = States.FIX_ANGLE;
                        pauseBetweenSteps();
                        runtime.reset();
                    }
                    break;
                case FIX_ANGLE:
                    logState("[FIX_ANGLE] fixing angle");

                    if(Math.abs(currentLocation.getAngle()) > 5 && runtime.seconds() < 2) {
                        fixAngle(currentLocation.getAngle());
                    } else {
                        mechDrive.Stop();
                        state = States.MOVE_TO_CORRECT_BUTTON;
                        pauseBetweenSteps();
                    }
                    break;
                case MOVE_TO_CORRECT_BUTTON: //move to correct button
                    logState("[MOVE_TO_CORRECT_BUTTON] move left or right to button");
                    //currentLocation = getCurrentLocation(visibleBeacon);
                    if(!targetButton.inRange(currentLocation.getX())){
                        moveToX(targetButton, currentLocation.getX(), currentLocation.getX());
                    } else {
                        mechDrive.Stop();
                        state = States.PRE_MOVE_IN;
                        pauseBetweenSteps();
                    }
                    break;

                case PRE_MOVE_IN: //press button begin
                    logState("[PRE_MOVE_IN] begin button press");
                    runtime.reset();
                    state = States.PRESS_BUTTON;
                    //pauseBetweenSteps();
                    break;
                case PRESS_BUTTON: //move into button
                    logState("[PRESS_BUTTON] move into button");
                    if(runtime.seconds() < 1.2) {
                        mechDrive.Drive(-0.2, 0, 0, false);
                    } else {
                        state = States.PRE_MOVE_OUT;

                        mechDrive.Stop();
                        pauseBetweenSteps();
                    }
                    break;
                case PRE_MOVE_OUT: //move away from button start
                    logState("[PRE_MOVE_OUT] begin stop button press");
                    runtime.reset();
                    state = States.BACK_OFF_BUTTON;
                    break;
                case BACK_OFF_BUTTON: //move away from button
                    logState("[BACK_OFF_BUTTON] move away from button");
                    if(runtime.seconds() < 1){
                        logState("[7.1] move backwards");
                        mechDrive.Drive(0.2, 0, 0, false);
                    } else {
                        logState("[7.2] stopping");
                        mechDrive.Stop();
                        pauseBetweenSteps();
                        state = States.DONE;
                    }
                    break;
                case DONE:
                    logState("[8] done");
                    //do nothing
                    break;

            }




            //Beacons are mirrored
           /* rightBeacon = beacon.getAnalysis().getStateLeft().toString();
            leftBeacon = beacon.getAnalysis().getStateRight().toString();

            robot.dashboard.displayPrintf(6, "Left: %s", leftBeacon);
            robot.dashboard.displayPrintf(7, "Right: %s", rightBeacon);*/



//            if(!atBeacon) {
//
//                for (VuforiaTrackable b : beacons) {
//
//                    VuforiaTrackableDefaultListener beacon = (VuforiaTrackableDefaultListener) b.getListener();
//
//
//
//                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) b.getListener()).getPose();
//
//
//                    if (pose != null) {
//                        VectorF translation = pose.getTranslation();
//
//                        double x = translation.get(0);
//                        double y = translation.get(1);      //not really used
//                        double z = translation.get(2);
//
//                        double angle = Math.toDegrees(Math.atan2(z, x)) + 90;
//
//                        robot.dashboard.displayPrintf(0, "****** " + b.getName());
//                        robot.dashboard.displayPrintf(1, "Turn " + angle + " degrees");
//                        robot.dashboard.displayPrintf(2, "X:" + x);
//                        robot.dashboard.displayPrintf(3, "Z:" + z);
//
//
//                        double h = 0, v = 0, r = 0;
//
//                        if (z < -150) {
//                            // move towards beacon, with  x centered
//                            moveToBeacon(x, z);
//                            robot.dashboard.displayPrintf(11, "Moving to beacon");
//                        } else if (!targetButton.inRange(x)) {
//                            robot.dashboard.displayPrintf(11, "Center on button");
//                            useLeftButton(x, z);
//                        } else {
//                            robot.dashboard.displayPrintf(11, "Where we want to be!");
//                            mechDrive.Stop();
//                        }
//
//                    }
//                }
//            }
//            else {
//
//                goStraight("press button", 1, true);
//                goStraight("back off", 2, false);
//
//            }

            telemetry.update();

            waitFor(0.02);
        }
    }


    private void logState(String msg){
        robot.dashboard.displayText(11, msg);
    }

    private void useLeftButton(double x, double z){
        moveToX(ButtonRange.LeftButton(), x, z);
    }
    private void useRightButton(double x, double z){
        moveToX(ButtonRange.RightButton(), x, z);
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
            r = 0.3;
        } else if(Math.abs((angle)) > 5) {
            r = 0.15;
        } else if(Math.abs((angle)) < 2.1) {
            r = 0;
        }


        if(angle > 0) {
            r = 0-r;
        }

        mechDrive.Drive(0, 0, r);
    }

    private void moveToBeacon(double x, double z, double angle)
    {
        double h=0,v=0,r=0;

        if (z < -610)
        {
            h = -0.3;
        }
        else if (z < -300)
        {
            h = -0.2;
        }
        else
        {
            h = -0.16;
        }


        if (x > 70) {
            v = 0.15;
        } else if (x < -70) {
            v = -0.15;
        } else {
            v = 0;
        }

        if(Math.abs(angle) > 10) {
            r = 0.1;
        } else if(Math.abs((angle)) > 5) {
            r = 0.05;
        } else if(Math.abs((angle)) < 2.1) {

            r = 0;
        }


        if(angle > 0) {
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

            double angle = Math.toDegrees(Math.atan2(z, x)) + 90;
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



