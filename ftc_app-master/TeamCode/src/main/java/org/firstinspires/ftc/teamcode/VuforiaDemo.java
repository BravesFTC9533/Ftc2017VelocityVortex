package org.firstinspires.ftc.teamcode;

/**
 * Created by doug on 11/22/2016.
 */
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Vuforia Demo")
@Disabled
public class VuforiaDemo extends LinearOpMode {


    private final float MM_PER_INCH = 25.4f;
    private final float ROBOT_WIDTH = 18*MM_PER_INCH;               // in mm
    private final float FTC_FIELD_WIDTH = (12*12 - 2)*MM_PER_INCH;  // in mm
    private final float TARGET_HEIGHT = 160.0f;                     // in mm


    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;

    VuforiaTrackable[] targets;
    VuforiaTrackableDefaultListener[] listeners;




    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    public static final String VUFORIA_KEY= "AeWceoD/////AAAAGWvk7AQGLUiTsyU4mSW7gfldjSCDQHX76lt9iPO5D8zaboG428rdS9WN0+AFpAlc/g4McLRAQIb5+ijFCPJJkLc+ynXYdhljdI2k9R4KL8t3MYk/tbmQ75st9VI7//2vNkp0JHV6oy4HXltxVFcEbtBYeTBJ9CFbMW+0cMNhLBPwHV7RYeNPZRgxf27J0oO8VoHOlj70OYdNYos5wvDM+ZbfWrOad/cpo4qbAw5iB95T5I9D2/KRf1HQHygtDl8/OtDFlOfqK6v2PTvnEbNnW1aW3vPglGXknX+rm0k8b0S7GFJkgl7SLq/HFNl0VEIVJGVQe9wt9PB6bJuxOMMxN4asy4rW5PRRBqasSM7OLl4W";


    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;


    public void runOpMode() throws InterruptedException {

        setupVuforia();
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        visionTargets.activate();


        while(opModeIsActive()){




            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
                telemetry.addData("Tracking " + trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible());




                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }
            }


            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));


            float[] coordinates = lastKnownLocation.getTranslation().getData();
            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;



            telemetry.update();
            idle();

        }

    }

    private Image getImageRGB() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforiaLocalizer.getFrameQueue().take();
        Image rgb = null;
        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        return rgb;
    }


    private String formatMatrix(OpenGLMatrix lastKnownLocation) {
        return lastKnownLocation.formatAsTransform();

    }


    public void setupVuforia (){
        parameters = new VuforiaLocalizer.Parameters(/*R.id.cameraMonitorViewId*/);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaLocalizer.setFrameQueueCapacity(1);
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");

        allTrackables.addAll(visionTargets);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        phoneLocation = createMatrix(0, 255, 0, 90, 0, 0);

        targets = new VuforiaTrackable[4];
        listeners = new VuforiaTrackableDefaultListener[4];

        createTarget(0, "Wheels Target", 12*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT, 90, 0, 0);
        createTarget(1, "Tools Target", -FTC_FIELD_WIDTH/2.0f, 30*MM_PER_INCH, TARGET_HEIGHT, 90, 0, 90);
        createTarget(2, "Legos Target", -30*MM_PER_INCH, FTC_FIELD_WIDTH/2.0f, TARGET_HEIGHT, 90, 0, 0);
        createTarget(3, "Gears Target", -FTC_FIELD_WIDTH/2.0f, -12*MM_PER_INCH, TARGET_HEIGHT, 90, 0, 90);

    }

    private void createTarget(int idx, String name, float x,float y,float z,float u,float v,float w) {
        VuforiaTrackable target = visionTargets.get(idx);
        target.setName(name);
        target.setLocation(createMatrix(x, y, z, u, v, w));

        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);

        targets[idx] = target;
        listeners[idx] = listener;
    }


    private OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w) {

        return  OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES, u, v, w));

    }
}
