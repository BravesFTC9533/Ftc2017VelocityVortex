package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcontroller.Util.*;
import org.firstinspires.ftc.robotcontroller.Util.Global;

/**
 * Created by TheSp on 1/14/2017.
 */

public class MMCompass {

    private double offset = 0;

    private double target;

    public double GetCurrentAngle() {
        double angle =  Global.compass + offset;
        return  angle;
    }

    public double GetTarget() {
        return  target + offset;
    }

    public void SetTargetDegrees(double degrees) {

        target =  degrees;
        if(target > 360) {
            offset = -180;
        } else if(target < 0) {
            offset = 180;
        }
        else {
            offset = 0;
        }
    }

}
