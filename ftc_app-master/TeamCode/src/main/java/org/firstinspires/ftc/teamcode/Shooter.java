package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

import ftclib.FtcDcMotor;
import ftclib.FtcServo;
import ftclib.FtcTouchSensor;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcTimer;

class Shooter //implements TrcTaskMgr.Task, TrcPidController.PidInput
{
    private enum ShooterState
    {
        LOAD_PARTICLE,
        ARM_AND_FIRE,
        PULL_BACK,
        DONE
    }   //enum ShooterState

    private String instanceName;
    private FtcDcMotor shooterMotor;
    private FtcServo ballGate;


    static final int COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder

    private TrcStateMachine<ShooterState> sm;
    private TrcTimer timer;
    private TrcEvent event;
    private TrcEvent completionEvent = null;
    private boolean continuousModeOn = false;

    private boolean enabled = false;

    Shooter(String instanceName)
    {
        this.instanceName = instanceName;

        shooterMotor = new FtcDcMotor("shooterMotor");
        shooterMotor.setInverted(false);
        shooterMotor.setBrakeModeEnabled(true);



        ballGate = new FtcServo("ballStop");
        ballGate.setPosition(RobotInfo.BALLGATE_DOWN_POSITION);

        sm = new TrcStateMachine<>(instanceName);
        timer = new TrcTimer(instanceName);
        event = new TrcEvent(instanceName);
    }

    private void setTaskEnabled(boolean enabled)
    {
        this.enabled = enabled;

//        if (enabled)
//        {
//            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
//            TrcTaskMgr.getInstance().registerTask(instanceName, this, TrcTaskMgr.TaskType.STOP_TASK);
//        }
//        else
//        {
//            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
//            TrcTaskMgr.getInstance().unregisterTask(this, TrcTaskMgr.TaskType.STOP_TASK);
//        }
    }

    public void stop()
    {
        if (sm.isEnabled())
        {
            sm.stop();
        }
        setTaskEnabled(false);
        continuousModeOn = false;

    }




    void setPower(double power)
    {
        shooterMotor.setPower(power);
    }

    void setBallGatePosition(double position)
    {
        ballGate.setPosition(position);
    }

    private void fire(ShooterState startState, boolean continuous, TrcEvent event)
    {
        continuousModeOn = continuous;
        this.completionEvent = event;
        if (sm.isEnabled())
        {
            stop();
        }
        sm.start(startState);
        setTaskEnabled(true);
    }

    void fireOneShot(TrcEvent event)
    {
        fire(ShooterState.ARM_AND_FIRE, false, event);
    }

    void fireOneShot()
    {
        fire(ShooterState.ARM_AND_FIRE, false, null);
    }

    void loadAndFireOneShot(TrcEvent event)
    {
        fire(ShooterState.LOAD_PARTICLE, false, event);
    }

    void loadAndFireOneShot()
    {
        fire(ShooterState.LOAD_PARTICLE, false, null);
    }

    void fireContinuous(boolean on)
    {
        continuousModeOn = on;
        if (on)
        {
            fire(ShooterState.LOAD_PARTICLE, true, null);
        }
    }

    //
    // Implements TrcTaskMgr.Task.
    //




    public void update()
    {
        //pidCtrl.displayPidInfo(3);

        if(!this.enabled) {
            return;
        }

        if (sm.isReady())
        {
            ShooterState state = sm.getState();

            switch (state)
            {
                case LOAD_PARTICLE:
                    //
                    // Flip the ball gate up to load a particle onto the shooter.
                    //
                    ballGate.setPosition(RobotInfo.BALLGATE_UP_POSITION);
                    timer.set(RobotInfo.SHOOTER_BALLGATE_OPEN_TIME, event);
                sm.waitForSingleEvent(event, ShooterState.ARM_AND_FIRE);
                break;

                case ARM_AND_FIRE:
                    //
                    // Flip the ball gate down and start turning the motor for the firing sequence.
                    //
                    ballGate.setPosition(RobotInfo.BALLGATE_DOWN_POSITION);

                    shooterMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shooterMotor.motor.setTargetPosition((int) shooterMotor.motor.getCurrentPosition() + COUNTS_PER_MOTOR_REV);
                    shooterMotor.setPower(RobotInfo.SHOOTER_POWER);


                    //timer.set();
                    //
                    // Check the touch sensor to see if the shooter has reached the firing point. If so, stop the
                    // motor for a brief moment.
                    //
//                    if (touchSensor.isActive())
//                    {
//                        shooterMotor.setPower(0.0);
//                        shooterMotor.resetPosition();
//                        timer.set(RobotInfo.SHOOTER_PAUSE_TIME, event);
//                        sm.waitForSingleEvent(event, ShooterState.PULL_BACK);
//                    }
                    break;

                case PULL_BACK:
                    //
                    // Pull back the shooter a little bit to prepare the next firing cycle. Set a timeout to prevent
                    // the shooter from being stuck not making target.
                    //

                    //sm.waitForSingleEvent(event, continuousModeOn? ShooterState.LOAD_PARTICLE: ShooterState.DONE);
                    break;

                default:
                case DONE:
                    //
                    // The firing cycle is done, notify whoever needs to know.
                    //
                    if (completionEvent != null)
                    {
                        completionEvent.set(true);
                        completionEvent = null;
                    }
                    sm.stop();
                    setTaskEnabled(false);
                    break;
            }
        }
    }



}   //class Shooter
