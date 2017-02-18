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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

/**
 * Created by Kerfuffle on 2/12/2017.
 */

@Autonomous(name = "RealAutonomous", group = "Autobot")
public class RealAutonomous extends MMOpMode_Linear {

    private VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaTrackables beacons;
    public static TeamColor teamColor = TeamColor.RED;
    public static int STRAFE = 0, FORWARD = 1;

    static final float      MM_PER_INCH             = 25.4f;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    double timePerRotationClockwiseMS = 4 * 1000.0;
    double timePerRotationCounterClockwiseMS = 4.1 * 1000.0;



    int leftFrontPosition = 0;
    int leftBackPosition = 0;
    int rightFrontPosition = 0;
    int rightBackPosition = 0;



    enum TeamColor {
        BLUE,
        RED
    }

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

    private VuforiaTrackableDefaultListener getBeacon(String name) {
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

        return new MMTranslation(translation);

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

    private void driveFor(double secs, double h, double v, double r) {
        runtime.reset();
        Drive(h, v, r);
        do {
        } while(opModeIsActive() && runtime.seconds() < secs);
    }

    private void driveTo(double dist, double h, double v, double r) {

        //21 5/8
        //21 1/2
        //21 1/8


        if(Math.abs(h) > 0 && v == 0) {
            dist *= 1.11;
        }

        robot.backLeftMotor.resetPosition();
        robot.backRightMotor.resetPosition();
        robot.leftMotor.resetPosition();
        robot.rightMotor.resetPosition();

//        leftBackPosition = robot.backLeftMotor.getPosition();
//        leftFrontPosition = robot.leftMotor.getCurrentPosition();
//        rightBackPosition = robot.backRightMotor.getCurrentPosition();
//        rightFrontPosition = robot.rightMotor.getCurrentPosition();



//        while (robot.backLeftMotor.getCurrentPosition() != 0 && robot)
//        {
//            robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

//        robot.leftMotor.setMaxSpeed(4000);
//        robot.rightMotor.setMaxSpeed(4000);
//        robot.backLeftMotor.setMaxSpeed(4000);
//        robot.backRightMotor.setMaxSpeed(4000);
//
//        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive())
        {
//            int backLeftTarget = Math.abs(robot.backLeftMotor.getCurrentPosition()) + (int)(dist*COUNTS_PER_INCH);          //-4000 , 1000   or    4000 , 7000      using abs limits direction
//            int frontLeftTarget = Math.abs(robot.leftMotor.getCurrentPosition()) + (int)(dist*COUNTS_PER_INCH);
//            int backRightTarget = Math.abs(robot.backRightMotor.getCurrentPosition()) + (int)(dist*COUNTS_PER_INCH);
//            int frontRightTarget = Math.abs(robot.rightMotor.getCurrentPosition()) + (int)(dist*COUNTS_PER_INCH);

            int target = (int)(dist*COUNTS_PER_INCH);
            mechDrive.Drive(h, v, r, false);
            do{
                robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(4, "frontLeft: " + robot.rightMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(5, "frontRight: " + robot.leftMotor.getPosition() + " target: " + target);

            }
            while ( opModeIsActive()
                    && (Math.abs(robot.backLeftMotor.getPosition()) < target)
                    && (Math.abs(robot.backRightMotor.getPosition()) < target)
                    && (Math.abs(robot.rightMotor.getPosition()) < target)
                    && (Math.abs(robot.leftMotor.getPosition()) < target));
            mechDrive.Stop();
        }

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
        } while(opModeIsActive()
                && runtime.seconds() < 10
//                && ((currentLocation.getX() > 50 || currentLocation.getX() < -50)
                && currentLocation.getZ() < -(8*MM_PER_INCH));

    }

    private void moveToBeacon(double x, double z, double angle) {
        double h=0,v=0,r=0;


        // slow movement into beacon as we get closer
        if (z < -650) {
            h = -0.35;
        } else if (z < -600) {
            h = -0.3;
        } else {
            h = -0.2;
        }

        //fix side to side movement

        if(x > 100 || x < -100)
        {
            v = 0.15;
        }
        else if(x > 50 || x < -50)
        {
            v = 0.1;
        }
        else if(x < 25 && x > -25)
        {
            v = 0.0;
        }

        robot.dashboard.displayText(10, ""+x);


        if(x < 0) {
            v *= -1;
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

    private void goForBeacon(VuforiaTrackableDefaultListener visibleBeacon) {
        ButtonRange targetButton = null;

        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);

        }

        Stop();
        waitFor(1);

        targetButton = getTargetButton(visibleBeacon);
        if(targetButton == null || targetButton.getName().equals(ButtonRange.Unknown().getName())){
            robot.dashboard.displayText(10, "UNABLE TO FIND TEAM COLOR");
            return;
        }

        robot.dashboard.displayText(13, "Beacon Config: " + targetButton.getName());

        if (opModeIsActive())
        {
            moveToBeacon(visibleBeacon); //move in closer
            Stop();
            waitFor(0.1);
        }

        // looks like its stopping after it moves towards the beacon (at the -500 mm mark)


        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);
            Stop();
            waitFor(0.1);
        }

        if (opModeIsActive())
        {
            if (targetButton.getName().equals("Left Button"))
            {
                driveTo(Math.abs(targetButton.getOffset()), 0, 0.25, 0);
                //waitFor(3000);
            }
            else if (targetButton.getName().equals("Right Button"))
            {
                driveTo(Math.abs(targetButton.getOffset()), 0, -0.25, 0);
            }
            else
            {

            }
            Stop();
        }

        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);
            Stop();
            waitFor(0.1);
        }

        MMTranslation currentLocation = getCurrentLocation(visibleBeacon);
        double inches = Math.abs(currentLocation.getZ() / MM_PER_INCH);

        if (opModeIsActive())
        {
            //move in to press button
            driveTo(inches, -0.2, 0, 0);

            //move out away from button
            driveTo(23, 0.6, 0, 0);
        }

        waitFor(0.2);

        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);

        }

    }


    /******************************************************************************************************************************************************************************
     * *****************************************************************************************************************************************************************************
     * *********************************************************************************************************
     */




    private void DriveOffWall() {
        driveTo(67, -0.9, -0.9, 0);
        waitFor(0.1);
    }
    private void Beacon1() {
        VuforiaTrackableDefaultListener visibleBeacon = null;

        // move over to right a lil
        driveTo(15, 0, -0.9, 0);
        Stop();
        waitFor(0.5);

        // find 1st picture
        runtime.reset();
        do {
            if (teamColor == TeamColor.RED)
            {
                visibleBeacon = getBeacon("gears");
            }
            else if (teamColor == TeamColor.BLUE)
            {
                visibleBeacon = getBeacon("wheels");
            }
        }while (opModeIsActive() && runtime.seconds() < 5 && visibleBeacon == null);


        if (opModeIsActive())
        {
            goForBeacon(visibleBeacon);
        }


    }
    private void Beacon2(boolean didBeacon1) {
        VuforiaTrackableDefaultListener visibleBeacon = null;

        double dist = 48;

        if(didBeacon1) {

        }

        if(teamColor == TeamColor.RED) {
            dist = (dist - lastTarget.getOffset());
        } else {
            dist = (dist + lastTarget.getOffset());
        }


//        if (teamColor == TeamColor.BLUE)
//        {
//            if (lastTarget.getName().equals("Left Button"))
//            {
//                dist -= 3;
//            }
//        }
//        else if (teamColor == TeamColor.RED)
//        {
//            if (lastTarget.getName().equals("Right Button"))
//            {
//                dist -= 3;
//            }
//        }


        driveTo(dist, 0, -0.9, 0);

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

        if (opModeIsActive())
        {
            goForBeacon(visibleBeacon);
        }
    }
    private void Shoot() {

    }
    private void Park() {

    }


    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        robot.dashboard.displayText(0, "*****WAAAAAIITT!!!!!  for it to say ready");
        boolean driveOffWall = true,
                beacon1 = true,
                beacon2 = true,
                shoot = false,
                park = false,
                test=false;

        if(!test) {
            initVuforia();

            if (!OpenCVLoader.initDebug()) {
                logState("Unable to initialize opencv");
                waitFor(0.25);
                //Logger.d("Internal OpenCV library not found. Using OpenCV Manager for initialization");
                //OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, null);
            } else {
                //Logger.d("OpenCV library found inside package. Using it!");
                //mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
            }
        }


        robot.dashboard.displayText(0, "Autonomous mode READY, waiting for start...");
        waitForStart();



        if (opModeIsActive())
        {
            if (driveOffWall)
            {
                logState("[DriveOffWall]");
                DriveOffWall();
            }
            if (beacon1)
            {
                logState("[Beacon 1]");
                Beacon1();
            }
            if (beacon2)
            {
                logState("[Beacon 2]");
                Beacon2(beacon1);
            }
            if (shoot)
            {
                logState("[Shoot]");
                Shoot();
            }
            if (park)
            {
                logState("[Park]");
                Park();
            }
            if(test){
                driveTo(24, 0.3, 0, 0);
            }
        }
    }

}
