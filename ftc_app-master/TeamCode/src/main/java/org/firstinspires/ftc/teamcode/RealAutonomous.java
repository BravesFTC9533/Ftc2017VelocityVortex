package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Util.ButtonRange;
import org.firstinspires.ftc.teamcode.Util.Helpers;
import org.firstinspires.ftc.teamcode.Util.OCVUtils;
import org.firstinspires.ftc.teamcode.Util.VortexUtils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

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

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    double timePerRotationClockwiseMS = 4 * 1000.0;
    double timePerRotationCounterClockwiseMS = 4.1 * 1000.0;

    enum TeamColor {
        BLUE,
        RED
    }

    public void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
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

            Mat mask = new Mat();

            Core.inRange(cropped, BEACON_BLUE_LOW, BEACON_BLUE_HIGH, mask);
            Moments mmnts = Imgproc.moments(mask, true);

            //calculating centroid of the resulting binary mask via image moments
            Log.i("CentroidX", "" + ((mmnts.get_m10() / mmnts.get_m00())));
            Log.i("CentroidY", "" + ((mmnts.get_m01() / mmnts.get_m00())));

            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2) {
                return VortexUtils.BEACON_RED_BLUE;
            } else {
                return VortexUtils.BEACON_BLUE_RED;
            }//else
        }//if

        return VortexUtils.NOT_VISIBLE;
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

    private void driveFor(double secs, double h, double v, double r)
    {
        runtime.reset();
        Drive(h, v, r);
        do {
        } while(opModeIsActive() && runtime.seconds() < secs);
    }

    private void driveTo(double dist, double h, double v, double r)
    {
        if (h != 0 && v == 0 && r == 0)
        {
            //dist+=5;
        }

        if (opModeIsActive())
        {
            int backLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
            int frontLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
            int backRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
            int frontRightTarget = robot.rightMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);


            mechDrive.Drive(h, v, r, false);
            do{
                robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getCurrentPosition() + " target: " + backLeftTarget);
                robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getCurrentPosition() + " target: " + backRightTarget);
                robot.dashboard.displayText(4, "frontLeft: " + robot.rightMotor.getCurrentPosition() + " target: " + frontLeftTarget);
                robot.dashboard.displayText(5, "frontRight: " + robot.leftMotor.getCurrentPosition() + " target: " + frontRightTarget);

            }
            while ((Math.abs(robot.backLeftMotor.getCurrentPosition()) < backLeftTarget) && (Math.abs(robot.backRightMotor.getCurrentPosition()) < backRightTarget) && (Math.abs(robot.rightMotor.getCurrentPosition()) < frontRightTarget) && (Math.abs(robot.leftMotor.getCurrentPosition()) < frontLeftTarget));
            mechDrive.Stop();

            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        } while(opModeIsActive() && currentLocation.getZ() < -500 && runtime.seconds() < 10);
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

    private void goForBeacon(VuforiaTrackableDefaultListener visibleBeacon)
    {
        ButtonRange targetButton = null;

        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);
            Stop();
            waitFor(1);
        }

        targetButton = getTargetButton(visibleBeacon);
        if(targetButton == null || targetButton == ButtonRange.Unknown()){
            robot.dashboard.displayText(10, "UNABLE TO FIND TEAM COLOR");
            return;
        }

        robot.dashboard.displayText(13, "Beacon Config: " + targetButton.getName());

        if (opModeIsActive())
        {
            moveToBeacon(visibleBeacon);
            Stop();
            waitFor(0.1);
        }

        // looks like its stopping after it moves towards the beacon (at the -500 mm mark)


        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);
            Stop();
            waitFor(0.1);

            robot.dashboard.displayText(6, "here");
        }

        if (opModeIsActive())
        {
            if (targetButton.getName().equals("Left Button"))
            {
                driveTo(4.13386, 0, 0.12, 0);
            }
            else if (targetButton.getName().equals("Right Button"))
            {
                driveTo(1.1811, 0, -0.12, 0);
            }
            Stop();
        }

        if (opModeIsActive())
        {
            fixAngles(visibleBeacon);
            Stop();
            waitFor(0.1);
        }

    }


    /******************************************************************************************************************************************************************************
     * *****************************************************************************************************************************************************************************
     * *********************************************************************************************************
     */




    private void DriveOffWall()
    {
        driveTo(67, -0.9, -0.9, 0);
        waitFor(0.1);
    }
    private void Beacon1()
    {
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
    private void Beacon2()
    {

    }
    private void Shoot()
    {

    }
    private void Park()
    {

    }


    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        robot.dashboard.displayText(0, "*****WAAAAAIITT!!!!!  for it to say ready");
        initVuforia();
        robot.dashboard.displayText(0, "Autonomous mode READY, waiting for start...");
        waitForStart();

        boolean DriveOffWall = true,
                Beacon1 = true,
                Beacon2 = false,
                Shoot = false,
                Park = false;

        if (opModeIsActive())
        {
            if (DriveOffWall)
            {
                logState("[DriveOffWall]");
                DriveOffWall();
            }
            if (Beacon1)
            {
                logState("[Beacon 1]");
                Beacon1();
            }
            if (Beacon2)
            {
                logState("[Beacon 2]");
                Beacon2();
            }
            if (Shoot)
            {
                logState("[Shoot]");
                Shoot();
            }
            if (Park)
            {
                logState("[Park]");
                Park();
            }
        }
    }

}
