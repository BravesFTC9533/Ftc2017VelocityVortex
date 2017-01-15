package org.firstinspires.ftc.teamcode.Util;

/**
 * Created by TheSp on 1/14/2017.
 */

public class ButtonRange {

    private int min;
    private int max;

    public int getMin(){
        return  min;
    }
    public int getMax() {
        return max;
    }

    public boolean inRange(double x) {
        return x >= min && x <= max;
    }


    public static ButtonRange LeftButton() {
        ButtonRange b = new ButtonRange();
        b.min = -120;
        b.max = -80;
        return  b;

    }
    public static ButtonRange RightButton(){
        ButtonRange b = new ButtonRange();
        b.min = 20;
        b.max = 70;
        return  b;
    }

}
