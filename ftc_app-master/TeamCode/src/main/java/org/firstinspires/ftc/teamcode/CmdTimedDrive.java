package org.firstinspires.ftc.teamcode;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdTimedDrive implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        DRIVE_BY_TIME,
        DONE
    }   //enum State

    private static final String moduleName = "CmdTimedDrive";

    private Robot robot;
    private double delay;
    private double driveTime;
    private double xDrivePower;
    private double yDrivePower;
    private double turnPower;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdTimedDrive(Robot robot, double delay, double driveTime, double xDrivePower, double yDrivePower, double turnPower)
    {
        this.robot = robot;
        this.delay = delay;
        this.driveTime = driveTime;
        this.xDrivePower = xDrivePower;
        this.yDrivePower = yDrivePower;
        this.turnPower = turnPower;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdTimedDrive

    //
    // Implements the TrcRobot.AutoStrategy interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");

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
                        sm.setState(State.DRIVE_BY_TIME);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_BY_TIME);
                    }
                    break;

                case DRIVE_BY_TIME:
                    //
                    // Drive the robot with the given power for a set amount of time.
                    //
                    robot.driveBase.mecanumDrive_Cartesian(xDrivePower, yDrivePower, turnPower);
                    timer.set(driveTime, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0);
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString(), 0.0, 0.0, 0.0);
        }

        return done;
    }   //cmdPeriodic

}   //class CmdTimedDrive
