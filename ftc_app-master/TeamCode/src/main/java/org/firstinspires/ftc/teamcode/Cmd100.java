package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;



public class Cmd100 implements TrcRobot.RobotCommand {

    private enum State {
        NEAR_START,
        TURN_TO_WALL,
        GOTO_WALL,
        MOVE_TO_NEAR_BEACON,
        MOVE_TO_FAR_BEACON,
        STOP_ROBOT,
        PARALLEL_WALL,
        FIND_NEAR_TARGET_PAUSE,
        FIND_NEAR_TARGET,
        FIND_FAR_TARGET_PAUSE,
        FIND_FAR_TARGET,

        ALIGN_WALL1,
        ALIGN_WALL2,
        ALIGN_WALL3,
        ALIGN_BUTTON_LEFT,
        ALIGN_BUTTON_RIGHT,
        PUSH_BUTTON,
        NEXT_BEACON,
        PAUSE,
        DONE
    }

    private static final String moduleName = "Cmd100";
    private Robot robot;
    private AltRyanOpFtcAuto.Alliance alliance;
    private int numberBeacons;
    private int numberParticles;

    private VuforiaVision vuforiaVision;

    private TrcEvent event;
    private TrcEvent timerEvent;
    private TrcTimer timer;

    private TrcStateMachine<State> sm;

    Cmd100(Robot robot,
           AltRyanOpFtcAuto.Alliance alliance,
           double delay,
           int numParticles,
           AltRyanOpFtcAuto.ParkOption parkOption,
           int numBeaconButtons) {

        this.robot = robot;
        this.numberBeacons = numBeaconButtons;
        this.numberParticles = numParticles;
        this.alliance = alliance;

        vuforiaVision = new VuforiaVision(robot);
        vuforiaVision.setEnabled(true);

        event = new TrcEvent(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timer");
        timer = new TrcTimer(moduleName);

        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.NEAR_START);
    }


    State gotoState = State.GOTO_WALL;

    public boolean cmdPeriodic(double elapsedTime) {


        boolean done = !sm.isEnabled();
        State state = sm.getState();
        if(state != State.NEAR_START) {
            robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");
        }

        if (robot.pidDrive.isActive())
        {
            robot.encoderXPidCtrl.displayPidInfo(4);
            robot.encoderYPidCtrl.displayPidInfo(6);
            robot.gyroPidCtrl.displayPidInfo(8);
        }

        if(sm.isReady()) {
            state = sm.getState();
            State nextState;// = State.NEAR_START;

            int target;
            VectorF targetPos;
            OpenGLMatrix robotPos;
            double xDistance = 0.0, yDistance = 0.0;
            boolean printStateInfo = true;
            switch (state){
                case NEAR_START:

                    //nextState = State.TURN_TO_WALL;
                    sm.setState(State.PAUSE);
                    break;

                case PAUSE:
                    timer.set(1, event);
                    sm.waitForSingleEvent(event, gotoState);
                    break;
                case STOP_ROBOT:
                    robot.driveBase.stop();
                    sm.setState(State.PAUSE);
                    break;
                case GOTO_WALL:
                    gotoState = State.TURN_TO_WALL;



                    robot.gyroPidCtrl.setNoOscillation(false);
                    robot.setPIDDriveTarget(0, -52, 0, false, event);
                    sm.waitForSingleEvent(event, State.PAUSE);

                    break;
                case TURN_TO_WALL:

                    robot.targetHeading = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE ? 45 : -45;


                    robot.gyroPidCtrl.reset();
                    //nextState =
                    gotoState = State.MOVE_TO_NEAR_BEACON;
                    //robot.gyroPidCtrl.setNoOscillation(true);

                    robot.setPIDDriveTarget(0, 0, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.PAUSE);
                    break;

                case MOVE_TO_NEAR_BEACON:
                    gotoState = State.FIND_NEAR_TARGET_PAUSE;
                    robot.encoderYPidCtrl.setNoOscillation(true);
                    robot.encoderXPidCtrl.setNoOscillation(true);
                    robot.gyroPidCtrl.setNoOscillation(true);
                    robot.setPIDDriveTarget(0, -13, 0, false, event);
                    sm.waitForSingleEvent(event, State.PAUSE);

//                    robot.driveBase.mecanumDrive_Cartesian(0, 0.3, 0, true);
//                    timer.set(0.5, event);
//                    sm.waitForSingleEvent(event, State.STOP_ROBOT);
                    break;
                case MOVE_TO_FAR_BEACON:
                    gotoState = State.FIND_FAR_TARGET_PAUSE;
                    robot.encoderYPidCtrl.setNoOscillation(true);
                    robot.setPIDDriveTarget(0, -48, 0, false, event);
                    sm.waitForSingleEvent(event, State.PAUSE);
                    break;
                case FIND_NEAR_TARGET_PAUSE:
                    timer.set(1.0, timerEvent);
                    sm.setState(State.FIND_NEAR_TARGET);
                    break;
                case FIND_NEAR_TARGET:
                    target = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE ?
                            VuforiaVision.TARGET_GEARS : VuforiaVision.TARGET_WHEELS;

                    if((targetPos = vuforiaVision.getTargetPosition(target)) != null) {

                        double xTargetDistance = targetPos.get(2) / RobotInfo.MM_PER_INCH;
                        xDistance = 52.0 - xTargetDistance;
                        yDistance = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE? 16.0: -16.0;
                        robotPos = vuforiaVision.getRobotLocation(target);

                        robot.dashboard.displayPrintf(3, "%s found at %d inches",
                                vuforiaVision.getTargetName(target),
                                (int) xTargetDistance);

                        robot.tracer.traceInfo("Auto40DF", "xDistance=%.2f, yDistance=%.2f, heading=%.1f",
                                xDistance, yDistance, robot.targetHeading);

                        robot.dashboard.displayPrintf(4, "RobotLoc: %s", robotPos.formatAsTransform());

                        gotoState = State.ALIGN_WALL1;
                        sm.setState(State.PAUSE);

                    } else if(timerEvent.isSignaled()) {
                        sm.setState(State.DONE);
                    }
                    break;
                case ALIGN_WALL1:

                    target = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE ?
                            VuforiaVision.TARGET_GEARS : VuforiaVision.TARGET_WHEELS;

                    VectorF angles = vuforiaVision.getAngles(target);
                    if(angles == null ){
                        sm.setState(State.DONE);
                    }
                    double angleToWall = (Math.toDegrees(angles.get(0)) + 270) % 360;
                    double angle = angleToWall - 90;

                    gotoState = State.MOVE_TO_FAR_BEACON;

                    robot.setPIDDriveTarget(0, 0, angle, false, event);
                    sm.waitForSingleEvent(event, State.PAUSE);

                    break;
                case FIND_FAR_TARGET_PAUSE:
                    timer.set(1.0, timerEvent);
                    sm.setState(State.FIND_FAR_TARGET);
                    break;
                case FIND_FAR_TARGET:
                    target = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE ?
                            VuforiaVision.TARGET_TOOLS : VuforiaVision.TARGET_LEGOS;

                    if((targetPos = vuforiaVision.getTargetPosition(target)) != null) {

                        double xTargetDistance = targetPos.get(2) / RobotInfo.MM_PER_INCH;
//                        double yTargetDistance = targetPos.get(0) / RobotInfo.MM_PER_INCH;
                        xDistance = 52.0 - xTargetDistance;
                        yDistance = alliance == AltRyanOpFtcAuto.Alliance.RED_ALLIANCE? 16.0: -16.0;
//                        yDistance = 6.0 - yTargetDistance;
                        robot.dashboard.displayPrintf(3, "%s found at %d inches",
                                vuforiaVision.getTargetName(target),
                                (int) xTargetDistance);
                        robot.tracer.traceInfo("Auto40DF", "xDistance=%.2f, yDistance=%.2f, heading=%.1f",
                                xDistance, yDistance, robot.targetHeading);
                        robot.textToSpeech.speak(String.format(Locale.US, "%s found at %d inches",
                                vuforiaVision.getTargetName(target),
                                (int) xTargetDistance),
                                TextToSpeech.QUEUE_FLUSH, null);

                        sm.setState(State.DONE);
//                        robot.gyroPidCtrl.setNoOscillation(true);
//
//                        robot.setPIDDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
//                        robot.gyroPidCtrl.setNoOscillation(true);
//                        sm.waitForSingleEvent(event, State.DONE);

                    } else if(timerEvent.isSignaled()) {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                    done = true;
                    sm.stop();
                    break;
            }
        }

        return done;
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
