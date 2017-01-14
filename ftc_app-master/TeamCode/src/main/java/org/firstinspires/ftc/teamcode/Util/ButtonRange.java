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

    public static ButtonRange LeftButton() {
        ButtonRange b = new ButtonRange();
        b.min = -150;
        b.max = -120;
        return  b;

    }
    public static ButtonRange RightButton(){
        ButtonRange b = new ButtonRange();
        b.min = 20;
        b.max = 70;
        return  b;
    }

}
