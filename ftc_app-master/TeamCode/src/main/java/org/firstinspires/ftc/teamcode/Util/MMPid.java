package org.firstinspires.ftc.teamcode.Util;

/**
 * Created by TheSp on 1/14/2017.
 */

public class MMPid {

    long lastTime;
    double Input, Output, Setpoint;
    double ITerm, lastInput;
    double ki, kp, kd;

    int SampleTime = 1000; //1 sec
    double outMin, outMax;
    boolean inAuto = false;

    ControlModes controllerDirection = ControlModes.DIRECT;

    enum ControlModes {
        DIRECT,
        REVERSE

    };


    enum Modes {
        MANUAL,
        AUTOMATIC
    }



    public void Compute(){
        if(!inAuto) return;;

        long now = System.currentTimeMillis() % 1000;
        long timeChange = (now - lastTime);
        if(timeChange >= SampleTime) {

            // compute all the working error variables
            double error = Setpoint - Input;
            ITerm += (ki * error);
            if(ITerm > outMax) {
                ITerm = outMax;
            }
            else if(ITerm < outMin) {
                ITerm = outMin;
            }
            double dInput = (Input - lastInput);

            //compute PID output
            Output = kp * error + ITerm - kd * dInput;
            if(Output > outMax)  {
                Output = outMax;
            } else if(Output < outMin){
                Output = outMin;
            }
            lastInput = Input;
            lastTime = now;

        }
    }

    public void SetTunings(double Kp, double Ki, double Kd){
        if(Kp < 0 || Ki < 0 || Kd < 0) return;;

        double SampleTimeInSec = ((double)SampleTime/ 1000);
        kp = Kp;
        ki = Ki * SampleTimeInSec;
        kd = Kd / SampleTimeInSec;

        if(controllerDirection == ControlModes.REVERSE) {
            kp = (0 - kp);
            ki = (0 - ki);
            kd = (0 - kd);
        }

    }

    public void SetSampleTime(int newSampleTime) {
        if(newSampleTime > 0 ){
            double ratio = (double)newSampleTime / (double)SampleTime;

            ki *= ratio;
            kd /= ratio;
            SampleTime = newSampleTime;
        }
    }

    public void SetOutputLimits(double min, double max) {
        if(min > max) return;;

        outMin = min;
        outMax = max;
        if(Output>outMax) {
            Output = outMax;
        }
        else if(Output < outMin) {
            Output = outMin;
        }

        if(ITerm > outMax) {
            ITerm = outMax;
        } else if(ITerm < outMin) {
            ITerm = outMin;
        }
    }

    public void SetMode(Modes mode) {
        boolean newAuto = (mode == Modes.AUTOMATIC);

        if(newAuto == !inAuto) {
            // we just went from manual to auto
            Initialize();
        }
        inAuto = newAuto;

    }

    public void Initialize (){
        lastInput = Input;
        ITerm = Output;
        if(ITerm > outMax ) {
            ITerm = outMax;
        } else if(ITerm < outMin)  {
            ITerm = outMin;
        }

    }

    public void SetControllerDirection(ControlModes mode) {
        controllerDirection = mode;
    }


}
