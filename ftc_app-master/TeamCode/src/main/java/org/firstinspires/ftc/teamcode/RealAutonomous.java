package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

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

/**
 * Created by Kerfuffle on 2/12/2017.
 */

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
        int backLeftTarget = robot.backLeftMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
        int frontLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
        int backRightTarget = robot.backRightMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);
        int frontRightTarget = robot.rightMotor.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        h = Helpers.clipMotorPower(h);
        v = Helpers.clipMotorPower(v);
        r = Helpers.clipMotorPower(r);

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
            frontLeft = Helpers.scalePower(frontLeft, max);
            frontRight = Helpers.scalePower(frontRight, max);
            backLeft = Helpers.scalePower(backLeft, max);
            backRight = Helpers.scalePower(backRight, max);
        }

        robot.leftMotor.setPower(frontLeft);
        robot.rightMotor.setPower(frontRight);
        robot.backRightMotor.setPower(backRight);
        robot.backLeftMotor.setPower(backLeft);
    }

    private void DriveOffWall()
    {

    }
    private void Beacon1()
    {

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

        boolean DriveOffWall = false,
                Beacon1 = false,
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
