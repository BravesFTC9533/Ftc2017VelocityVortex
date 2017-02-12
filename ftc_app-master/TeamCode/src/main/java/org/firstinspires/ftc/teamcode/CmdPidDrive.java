package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdPidDrive implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = false;
    private static final boolean debugYPid = false;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        DO_DELAY,
        DO_PID_DRIVE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdPidDrive";

    private Robot robot;
    private double delay;
    private double xDistance;
    private double yDistance;
    private double heading;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdPidDrive(Robot robot, double delay, double xDistance, double yDistance, double heading)
    {
        this.robot = robot;
        this.delay = delay;
        this.xDistance = xDistance;
        this.yDistance = yDistance;
        this.heading = heading;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = false; //!sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null? sm.getState().toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.DO_PID_DRIVE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DO_PID_DRIVE);
                    }
                    break;

                case DO_PID_DRIVE:
                    //
                    // Drive the set distance and heading.
                    //
                    robot.gyroPidCtrl.setNoOscillation(false);
                    robot.setPIDDriveTarget(xDistance, yDistance, heading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), xDistance, yDistance, heading);
        }

        if (robot.pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            robot.tracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                    robot.battery.getCurrentVoltage(), robot.battery.getLowestVoltage());

            if (debugXPid && xDistance != 0.0)
            {
                robot.encoderXPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugYPid && yDistance != 0.0)
            {
                robot.encoderYPidCtrl.printPidInfo(robot.tracer);
            }

            if (debugTurnPid)
            {
                robot.gyroPidCtrl.printPidInfo(robot.tracer);
            }
        }

        if(done == true) {
            robot.dashboard.displayText(3, "finished");
        }
        return done;
    }   //cmdPeriodic



}   //class CmdPidDrive
