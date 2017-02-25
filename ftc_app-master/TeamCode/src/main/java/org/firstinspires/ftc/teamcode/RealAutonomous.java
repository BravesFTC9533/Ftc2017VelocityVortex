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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

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

import java.util.Arrays;


import static org.firstinspires.ftc.teamcode.Util.VortexUtils.BEACON_BLUE_HIGH;
import static org.firstinspires.ftc.teamcode.Util.VortexUtils.BEACON_BLUE_LOW;
import static org.firstinspires.ftc.teamcode.Util.VortexUtils.getImageFromFrame;

/**
 * Created by Kerfuffle on 2/12/2017.
 */

@Autonomous(name = "RealAutonomous", group = "Autobot")
public class RealAutonomous extends MMOpMode_Linear {

    private ElapsedTime runtime = new ElapsedTime();
    VuforiaVision vuforiaVision = null;



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

    static final int MAX_ROBOT_SPEED = 3500;

    //vuforia pause
    static final double VUFORIA_PAUSE_TO_FIND_COLOR = 1.0;
    static final double VUFORIA_PAUSE_TO_CENTER = 0.5;

    //inches per count
    double ENCODER_Y_INCHES_PER_COUNT = 0.008726388889;
    double ENCODER_X_INCHES_PER_COUNT = 0.007953739871; //0.008180989581; // 0.007817390046;

    enum TeamColor {
        BLUE,
        RED
    }

    private void initVuforia() {

        vuforiaVision = new VuforiaVision(robot);
        vuforiaVision.setEnabled(true);

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
        robot.dashboard.displayPrintf(1, msg, args);
    }

    private void driveTo(double dist, double h, double v, double r) {

        //21 5/8
        //21 1/2
        //21 1/8

        if(teamColor == TeamColor.BLUE) {
            v *= -1;
        }

//        if(Math.abs(h) > 0 && v == 0) {
//            dist *= 1.11;
//        }

        robot.backLeftMotor.resetPosition();
        robot.backRightMotor.resetPosition();
        robot.leftMotor.resetPosition();
        robot.rightMotor.resetPosition();

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setMaxSpeed(MAX_ROBOT_SPEED);

        if (opModeIsActive())
        {
            int target = 0;

            if(v != 0 && h == 0) {
                target = (int)(dist / ENCODER_Y_INCHES_PER_COUNT);
            } else if(h != 0 && v == 0) {
                target = (int)(dist / ENCODER_X_INCHES_PER_COUNT);
            } else {
                target = (int)(dist*COUNTS_PER_INCH);
            }


            do{
                robot.dashboard.displayText(2, "backLeft: " + robot.backLeftMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(3, "backRight: " + robot.backRightMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(4, "frontLeft: " + robot.rightMotor.getPosition() + " target: " + target);
                robot.dashboard.displayText(5, "frontRight: " + robot.leftMotor.getPosition() + " target: " + target);
                scaleDrive(target, h, v);


        }
            while ( opModeIsActive()
                    && (Math.abs(robot.backLeftMotor.getPosition()) < target)
                    && (Math.abs(robot.backRightMotor.getPosition()) < target)
                    && (Math.abs(robot.rightMotor.getPosition()) < target)
                    && (Math.abs(robot.leftMotor.getPosition()) < target));
            mechDrive.Stop();
        }

    }

    private void scaleDrive(int target, double h, double v) {

        if(h == 0 && v != 0) {
            scaleDriveV(target, v);
        } else if(v == 0 && h != 0) {
            scaleDriveH(target, h);
        } else {
            mechDrive.Drive(h, v, 0, false);
        }

    }


    private double getScalePower(double target, double pos, double power) {


        int encoderPositionAtFullSpeed = 250;

        //ramp up speed
        if(pos < encoderPositionAtFullSpeed) {
            if(power > 0) {
                return Range.clip(pos / encoderPositionAtFullSpeed, 0.10, power);
            } else {
                return Range.clip((pos / encoderPositionAtFullSpeed) * -1, -1, -0.10);
            }
        }


        //now that we've ramped up, ramp down towards end
        double percent = 0.0 ;


        if(pos > 0) {
            percent = pos / target;
        }



        // from 80% to 100%, slowly ramp down
        //


        if(percent >= 0.8) {


            double val = (1-percent) * 5; // scale last 20% up to 100%

            if(power > 0) {
                return Range.clip(power * val, 0.3, 1);
            } else {
                return Range.clip((power * val * -1), -1, -0.3);
            }


        } else {
            return power;
        }
    }

    private void scaleDriveV(int target, double v){
        double pos = Math.abs(robot.leftMotor.getPosition());

        double scalePower = getScalePower(target, pos, v);


        mechDrive.Drive(0, scalePower, 0, false);

    }

    private void scaleDriveH(int target, double h){
        double pos = Math.abs(robot.leftMotor.getPosition());

        double scalePower = getScalePower(target, pos, h);

        mechDrive.Drive(scalePower, 0, 0, false);
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

        mechDrive.turn(angle, 1);

//        double ticksPerDegree_bl = 26.08055556;
//        double ticksPerDegree_br = 26.12222222;
//
//        double ticksPerDegree_fl = 27.36388889;
//        double ticksPerDegree_fr = 27.39166667;
//
//
//
//        double target_bl = Math.abs(ticksPerDegree_bl*angle);
//        double target_br = Math.abs(ticksPerDegree_br*angle);
//        double target_fl = Math.abs(ticksPerDegree_fl*angle);
//        double target_fr = Math.abs(ticksPerDegree_fr*angle);
//
//        double turnPower = 0.5;
//
//        //right negative is pos angle;
//
//
//
//
//        robot.resetPosition();
//
//        boolean rightIsNegative = angle > 0;
//
//        double leftPower = rightIsNegative ? turnPower : turnPower * -1;
//        double rightPower = rightIsNegative ? turnPower*-1 : turnPower;
//
//
//
//        double flp = robot.leftMotor.getPosition();
//        double frp = robot.rightMotor.getPosition();
//        double blp = robot.backLeftMotor.getPosition();
//        double brp = robot.backRightMotor.getPosition();
//
//        robot.dashboard.displayPrintf(9, "FL: t: %.3f, p: %.2f, pow: %.2f", target_fl, flp, leftPower);
//        robot.dashboard.displayPrintf(10, "FR: t: %.3f, p: %.2f, pow: %.2f", target_fr, frp, rightPower);
//
//        robot.dashboard.displayPrintf(12, "BL: t: %.3f, p: %.2f, pow: %.2f", target_bl, blp, leftPower);
//        robot.dashboard.displayPrintf(13, "BR: t: %.3f, p: %.2f, pow: %.2f", target_br, brp, rightPower);
//
//
//
//        robot.dashboard.displayPrintf(14, "about to run");
//
//
//        ElapsedTime time = new ElapsedTime();
//
//        while (opModeIsActive()
//                && Math.abs(flp) <= target_fl
//                && Math.abs(frp) <= target_fr
//                && Math.abs(blp) <= target_bl
//                && Math.abs(brp) <= target_br
//                && time.seconds() < 1
//                ) {
//
//
//
//            flp = robot.leftMotor.getPosition();
//            frp = robot.rightMotor.getPosition();
//            blp = robot.backLeftMotor.getPosition();
//            brp = robot.backRightMotor.getPosition();
//
//            robot.dashboard.displayPrintf(14, "running");
//            robot.dashboard.displayPrintf(9, "FL: t: %.3f, p: %.2f, pow: %.2f", target_fl, flp, leftPower);
//            robot.dashboard.displayPrintf(10, "FR: t: %.3f, p: %.2f, pow: %.2f", target_fr, frp, rightPower);
//
//            robot.dashboard.displayPrintf(12, "BL: t: %.3f, p: %.2f, pow: %.2f", target_bl, blp, leftPower);
//            robot.dashboard.displayPrintf(13, "BR: t: %.3f, p: %.2f, pow: %.2f", target_br, brp, rightPower);
//
//
//            robot.leftMotor.setPower(leftPower);
//            robot.backLeftMotor.setPower(leftPower);
//            robot.rightMotor.setPower(rightPower);
//            robot.backRightMotor.setPower(rightPower);
//
//
//        }
//        Stop();
//
//        robot.dashboard.displayPrintf(14, "Stopped");



    }
    private void turn_time(double angle) {
        double timeForTurn = 0.0;

        if(angle < 0) {
//            if(angle > -2){
//                angle -= 1;
//            }
            timeForTurn = (timePerRotationCounterClockwiseMS / 360) * Math.abs(angle);

        } else {
//            if(angle < 2) {
//                angle += 1;
//            }
            timeForTurn = (timePerRotationClockwiseMS / 360) * Math.abs(angle);
        }


        robot.dashboard.displayPrintf(15, "Time for turn: %f", timeForTurn);
        double rotationSpeed = 0.3;


        if(angle < 0) {
            rotationSpeed *= -1;
        }

        runtime.reset();
        if(opModeIsActive()) {
            Drive(0, 0, rotationSpeed);

            while (opModeIsActive() && runtime.milliseconds() < timeForTurn) {
                robot.dashboard.displayPrintf(14, "Runtime: %f", runtime.milliseconds());
            }
        }
        Stop();
    }

    private void fixAngles(VuforiaTrackableDefaultListener visibleBeacon){
        logState("[SQUARE UP TO WALL]");
        //VuforiaTrackableDefaultListener visibleBeacon = null;
        MMTranslation angles = null;
        MMTranslation angles2 = null;

        double angleToWall = 0;

        double angleToWall2 = 0;

        //visibleBeacon = getVisibleBeacon();
        if(visibleBeacon == null){
            //uh oh
            logState("Unable to locate beacon");
            Stop();
            return;

        }


        angles = anglesFromTarget(visibleBeacon);

        waitFor(0.25);
        angles2 = anglesFromTarget(visibleBeacon);


        if (angles == null && angles2 == null)
        {
            return;
        }

        if(angles != null) {
            angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
        }
        if(angles2 != null){
            angleToWall2 = (Math.toDegrees(angles.getX()) + 270) % 360;
        }



        double angle = angleToWall - 90;
        double angle2 = angleToWall2 - 90;


        if(Math.abs(angle - angle2) > 10) {
            //ok, this is too big a difference
            waitFor(0.5);
            angles = anglesFromTarget(visibleBeacon);
            if (angles == null )
            {
                return;
            }

            angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
            angle = angleToWall - 90;
        }

        logState("[SQUARE UP TO WALL] Angle: %f", angle);

        if(Math.abs(angle) >= 1) {
            turn(angle);
        }


    }

    public int getBeaconColor(VuforiaTrackableDefaultListener listener) {
        int color = -1;
        boolean usingFrontCamera = true;

        try {
            Image img = getImageFromFrame(vuforiaVision.getFrame(), PIXEL_FORMAT.RGB565);

            runtime.reset();

            do {
                color = getBeaconConfig(img, listener, vuforiaVision.getCameraCalibration()); //TODO
            }
            while (opModeIsActive() && runtime.seconds() < 5 && color == -1);

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

//    private void moveToBeacon(VuforiaTrackableDefaultListener visibleBeacon) {
//        logState("[MOVE_TO_BEACON] Move closer to beacon");
//
//        //VuforiaTrackableDefaultListener visibleBeacon = null;
//        MMTranslation angles;
//        MMTranslation currentLocation = null;
//        double angleToWall;
//
//        angles = anglesFromTarget(visibleBeacon);
//        runtime.reset();
//        do {
//            //visibleBeacon = getVisibleBeacon();
//            if(visibleBeacon == null){
//                Stop();
//                logState("Unable to find beacon");
//                return;
//            }
//            currentLocation = getCurrentLocation(visibleBeacon);
//
//            angleToWall = (Math.toDegrees(angles.getX()) + 270) % 360;
//            moveToBeacon(currentLocation.getX(), currentLocation.getZ(), angleToWall - 90);
//        } while(opModeIsActive()
//                && runtime.seconds() < 10
////                && ((currentLocation.getX() > 50 || currentLocation.getX() < -50)
//                && currentLocation.getZ() < -(8*MM_PER_INCH));
//
//    }

//    private void moveToBeacon(double x, double z, double angle) {
//        double h=0,v=0,r=0;
//
//
//        // slow movement into beacon as we get closer
//        if (z < -650) {
//            h = -0.6;
//        } else if (z < -600) {
//            h = -0.5;
//        } else {
//            h = -0.3;
//        }
//
//        //fix side to side movement
//
//        if(x > 100 || x < -100)
//        {
//            v = 0.15;
//        }
//        else if(x > 50 || x < -50)
//        {
//            v = 0.1;
//        }
//        else if(x < 25 && x > -25)
//        {
//            v = 0.0;
//        }
//
//        robot.dashboard.displayText(10, ""+x);
//
//
//        if(x < 0) {
//            v *= -1;
//        }
//
//
//        if(Math.abs(angle) > 10) {
//            r = 0.1;
//        }
//        if (Math.abs((angle)) > 5) {
//            r = 0.05;
//        } else if (Math.abs((angle)) < 2.1) {
//            r = 0;
//        }
//
//        if(angle < 0) {
//            r = 0-r;
//        }
//
//        if (opModeIsActive())
//        {
//            Drive(h, v, r);
//        }
//
//    }


    private void centerOnBeacon(MMTranslation currentLocation) {
        if(Math.abs(currentLocation.getX() + 20) > 10) {

            //we need to move!
            double inches = Math.abs(currentLocation.getX() + 20) / MM_PER_INCH;

            logState("[Center on beacon] NEED TO CORRECT x: %f, inches: %f", currentLocation.getX(), inches);

            double power = (currentLocation.getX() + 20) > 0 ? 0.3 : -0.3;
            if(teamColor == TeamColor.BLUE) {
                power *= -1;
            }
            driveTo(inches, 0, power, 0);
            pauseBetweenSteps();
        }

    }

    private void goForBeacon(VuforiaTrackableDefaultListener visibleBeacon, boolean fixInitialAngle) {
        ButtonRange targetButton = null;

        if (opModeIsActive() && fixInitialAngle)
        {
            logState("Fixing angle..");
            fixAngles(visibleBeacon);
        }

        waitFor(VUFORIA_PAUSE_TO_FIND_COLOR);


        targetButton = getTargetButton(visibleBeacon);
        if(targetButton == null || targetButton.getName().equals(ButtonRange.Unknown().getName())){
            robot.dashboard.displayText(10, "UNABLE TO FIND TEAM COLOR");
            return;
        }


        robot.dashboard.displayText(13, "Beacon Config: " + targetButton.getName());
        MMTranslation currentLocation = getCurrentLocation(visibleBeacon);
        centerOnBeacon(currentLocation);


        waitFor(VUFORIA_PAUSE_TO_CENTER);


        if (opModeIsActive())
        {
            logState("Fixing angle..");
            fixAngles(visibleBeacon);
            Stop();
            waitFor(VUFORIA_PAUSE_TO_CENTER);
        }


        //calculate how far away in inches from image
        currentLocation = getCurrentLocation(visibleBeacon);
        double inches = Math.abs(currentLocation.getZ() / MM_PER_INCH) - 4;


        //pauseBetweenSteps();

        if (opModeIsActive())
        {
            logState("Driving into beacon %f inches", inches);
            //move in to press button
            driveTo(inches / 2, -0.7, 0, 0);
            //pauseBetweenSteps();
            waitFor(VUFORIA_PAUSE_TO_CENTER);

            fixAngles(visibleBeacon);

            waitFor(VUFORIA_PAUSE_TO_CENTER);

            currentLocation = getCurrentLocation(visibleBeacon);

            centerOnBeacon(currentLocation);
            waitFor(VUFORIA_PAUSE_TO_CENTER);


            fixAngles(visibleBeacon);


            currentLocation = getCurrentLocation(visibleBeacon);

            inches = Math.abs(currentLocation.getZ() / MM_PER_INCH) - 3.5;
            driveTo(inches, -0.6, 0, 0);
            pauseBetweenSteps();

            logState("Pushing button..");
            pushButton(targetButton);

            pauseBetweenSteps();


            logState("Resetting button pusher");
            resetPusher();
            //move out away from button
            logState("Driving out from beacon");
            driveTo(4, 0.8, 0, 0);


            //pauseBetweenSteps();

            //waitFor(VUFORIA_PAUSE_TO_CENTER);
            //fixAngles(visibleBeacon);

            pauseBetweenSteps();

            //driveTo(17, 0.6, 0, 0);
            //fixAngles(visibleBeacon);
        }

        pauseBetweenSteps();


    }



    private void resetPusher(){
        robot.buttonPusher.setPosition(0.5);
    }

    private void pushButton(ButtonRange targetButton){
        if (targetButton.getName().equals("Left Button")) {

            robot.buttonPusher.setPosition(0);

        } else {
            robot.buttonPusher.setPosition(1);
        }
    }
    /******************************************************************************************************************************************************************************
     * *****************************************************************************************************************************************************************************
     * *********************************************************************************************************
     */


    private void pauseBetweenSteps(){
        //logPath("pausing waiting 2 seconds");
        waitFor(0.1);
    }

    private void DriveOffWall() {
        double length = teamColor == TeamColor.BLUE ? 60 : 44;


        driveTo(length, 0, -1, 0);
        //driveTo(length, -0.7, -0.7, 0);
        waitFor(0.1);
        turn(45);
        length = teamColor == TeamColor.BLUE ? 21 : 14;
        driveTo(length, 0, -0.9, 0);
        Stop();

        waitFor(0.5);

    }


    private void Beacon1() {
        VuforiaTrackableDefaultListener visibleBeacon = null;


        int target = teamColor == TeamColor.RED ? VuforiaVision.TARGET_GEARS : VuforiaVision.TARGET_WHEELS;

        // find 1st picture
        runtime.reset();
        do {
            visibleBeacon = vuforiaVision.getBeacon(target);

        }while (opModeIsActive() && runtime.seconds() < 5 && visibleBeacon == null);


        if (opModeIsActive())
        {
            goForBeacon(visibleBeacon, false);
        }


    }
    private void Beacon2(boolean didBeacon1) {
        VuforiaTrackableDefaultListener visibleBeacon = null;

        double dist = 44;



        //drive half way
        driveTo(dist / 2, 0, -1, 0);

        //fix angle by smashing wall
        driveTo(6, -1, 0, 0);
        driveTo(24, 1, 0, 0);


        //drive other half
        driveTo(dist / 2, 0, -1, 0);

        waitFor(1);
        runtime.reset();

        int target = teamColor == TeamColor.RED ? VuforiaVision.TARGET_TOOLS : VuforiaVision.TARGET_LEGOS;

        do {

            visibleBeacon = vuforiaVision.getBeacon(target);


        }while(opModeIsActive() && runtime.seconds() < 5 && visibleBeacon == null);

        if (opModeIsActive())
        {
            goForBeacon(visibleBeacon, true);
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
                test= false,
                test2 = false;

        robot.setBrakeModeEnabled(true);

        resetPusher();

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
                pauseBetweenSteps();
            }
            if (beacon1)
            {
                logState("[Beacon 1]");
                Beacon1();
                pauseBetweenSteps();
            }
            if (beacon2)
            {
                logState("[Beacon 2]");
                Beacon2(beacon1);
                pauseBetweenSteps();
            }
            if (shoot)
            {
                logState("[Shoot]");
                Shoot();
                pauseBetweenSteps();
            }
            if (park)
            {
                logState("[Park]");
                Park();
                pauseBetweenSteps();
            }
            if(test){
                driveTo(48*2, 1, 0, 0);
            }
            if(test2) {

                while(opModeIsActive()) {

//                    VectorF targetPos;
//                    OpenGLMatrix robotPos;
//
                    int target = VuforiaVision.TARGET_GEARS;
                    VuforiaTrackableDefaultListener visibleBeacon = vuforiaVision.getBeacon(target);
                    fixAngles(visibleBeacon);

                    waitFor(2);
//
//                    targetPos = vuforiaVision.getTargetPosition(target);
//                    //robotPos = vuforiaVision.getRobotLocation(target);
//
//
//                    if(targetPos == null){
//                        idle();
//                        continue;
//                    }
//                    double xTargetDistance = targetPos.get(2) / RobotInfo.MM_PER_INCH;
//                    double xDistance = 52.0 - xTargetDistance;
//                    //double yDistance = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE? 16.0: -16.0;
//                    robotPos = vuforiaVision.getRobotLocation(target);
//
//                    robot.dashboard.displayPrintf(3, "%s found at %d inches",
//                            vuforiaVision.getTargetName(target),
//                            (int) xTargetDistance);

                }
            }
        }
    }

}
