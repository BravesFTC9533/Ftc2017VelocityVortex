package org.firstinspires.ftc.teamcode;

import com.vuforia.CameraCalibration;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

//import FtcSampleCode.R;
import java.util.Vector;

import ftclib.FtcVuforia;

public class VuforiaVision
{
    public static final int TARGET_WHEELS = 0;
    public static final int TARGET_TOOLS = 1;
    public static final int TARGET_LEGOS = 2;
    public static final int TARGET_GEARS = 3;

    private Hardware9533 robot;
    private FtcVuforia vuforia;

    public VuforiaVision(Hardware9533 robot)
    {
        final float ROBOT_WIDTH = 18*RobotInfo.MM_PER_INCH;                 // in mm
        final float FTC_FIELD_WIDTH = (12*12 - 2)*RobotInfo.MM_PER_INCH;    // in mm
        final float TARGET_HEIGHT = 160.0f;                                 // in mm
        final String VUFORIA_LICENSE_KEY =
                "AeWceoD/////AAAAGWvk7AQGLUiTsyU4mSW7gfldjSCDQHX76lt9iPO5D8zaboG428rdS9WN0+AFpAlc/g4McLRAQIb5+ijFCPJJkLc+ynXYdhljdI2k9R4KL8t3MYk/tbmQ75st9VI7//2vNkp0JHV6oy4HXltxVFcEbtBYeTBJ9CFbMW+0cMNhLBPwHV7RYeNPZRgxf27J0oO8VoHOlj70OYdNYos5wvDM+ZbfWrOad/cpo4qbAw5iB95T5I9D2/KRf1HQHygtDl8/OtDFlOfqK6v2PTvnEbNnW1aW3vPglGXknX+rm0k8b0S7GFJkgl7SLq/HFNl0VEIVJGVQe9wt9PB6bJuxOMMxN4asy4rW5PRRBqasSM7OLl4W";

//                "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
//                        "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
//                        "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
//                        "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
        final int CAMERAVIEW_ID = R.id.cameraMonitorViewId;
        final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.FRONT;
        final String TRACKABLES_FILE = "FTC_2016-17";
        //
        // Note that the order of the targets must match the order in the FTC_2016-17.xml file.
        //
        final FtcVuforia.Target[] targets =
                {
                        //
                        // Blue alliance near beacon.
                        //
                        new FtcVuforia.Target(
                                "wheels", 90.0f, 0.0f, 0.0f, 12.0f*RobotInfo.MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT),
                        //
                        // Red alliance far beacon.
                        //
                        new FtcVuforia.Target(
                                "tools", 90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, 30.0f*RobotInfo.MM_PER_INCH, TARGET_HEIGHT),
                        //
                        // Blue alliance far beacon.
                        //
                        new FtcVuforia.Target(
                                "legos", 90.0f, 0.0f, 0.0f, -30.0f*RobotInfo.MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT),
                        //
                        // Red alliance near beacon.
                        //
                        new FtcVuforia.Target(
                                "gears", 90.0f, 0.0f, 90.0f, -FTC_FIELD_WIDTH/2.0f, -12.0f*RobotInfo.MM_PER_INCH, TARGET_HEIGHT)
                };

        this.robot = robot;
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR, TRACKABLES_FILE, targets.length);
        //
        // Phone location: Mounted on the left side center of the robot with the front camera facing outward.
        //
        OpenGLMatrix phoneLocationOnRobot = vuforia.locationMatrix(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH/2.0f, 0.0f);
        vuforia.setTargets(targets, phoneLocationOnRobot);
    }   //VuforiaVision

    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    public String getTargetName(int index)
    {
        return vuforia.getTarget(index).getName();
    }   //getTargetName

    public VuforiaLocalizer.CloseableFrame getFrame() {
        return vuforia.getFrame();
    }
    public CameraCalibration getCameraCalibration() {
        return  vuforia.getCameraCalibration();
    }

    public VectorF getAngles(int index){

        VuforiaTrackable target = vuforia.getTarget(index);
        VectorF angles = vuforia.getAngles(target);
        return angles;
    }

    public VuforiaTrackableDefaultListener getBeacon(int index){
        VuforiaTrackableDefaultListener target = (VuforiaTrackableDefaultListener)vuforia.getTarget(index).getListener();
        return  target;
    }

    public VectorF getTargetPosition(int index)
    {
        VectorF targetPos = null;
        VuforiaTrackable target = vuforia.getTarget(index);

        if (vuforia.isTargetVisible(target))
        {
            OpenGLMatrix pose = vuforia.getTargetPose(target);
            if (pose != null)
            {
                targetPos = pose.getTranslation();
                robot.dashboard.displayPrintf(1, "%s: %6.2f,%6.2f,%6.2f",
                        target.getName(),
                        targetPos.get(0)/RobotInfo.MM_PER_INCH,
                        targetPos.get(1)/RobotInfo.MM_PER_INCH,
                        targetPos.get(2)/RobotInfo.MM_PER_INCH);
//                robot.tracer.traceInfo("TargetPos", "%s: %6.2f, %6.2f, %6.2f",
//                        target.getName(),
//                        targetPos.get(0)/RobotInfo.MM_PER_INCH,
//                        targetPos.get(1)/RobotInfo.MM_PER_INCH,
//                        targetPos.get(2)/RobotInfo.MM_PER_INCH);
            }
        }

        return targetPos;
    }   //getTargetPosition

    public OpenGLMatrix getRobotLocation(int index)
    {
        VuforiaTrackable target = vuforia.getTarget(index);
        OpenGLMatrix robotLocation = vuforia.getRobotLocation(target);

        if (robotLocation != null)
        {
            robot.dashboard.displayPrintf(2, "RobotLoc: %s", robotLocation.formatAsTransform());
            //robot.tracer.traceInfo("RobotLoc", "%s: %s", target.getName(), robotLocation.formatAsTransform());
        }

        return robotLocation;
    }   //getRobotLocation

}   //class VuforiaVision
