package org.firstinspires.ftc.teamcode.Util;

/**
 * Created by TheSp on 1/14/2017.
 */

public class ButtonRange {

    private int min;
    private int max;

    private String name;
    public int getMin(){
        return  min;
    }
    public int getMax() {
        return max;
    }


    public String getName() { return  name; }

    public boolean inRange(double x) {
        return x >= min && x <= max;
    }


    public static ButtonRange LeftButton() {
        ButtonRange b = new ButtonRange();
        b.min = -100;
        b.max = -60;
        b.name = "Left Button";
        return  b;

    }
    public static ButtonRange RightButton(){
        ButtonRange b = new ButtonRange();
        b.min = 25;
        b.max = 60;
        b.name = "Right Button";
        return  b;
    }

    public static ButtonRange Unknown() {
        ButtonRange b = new ButtonRange();
        b.min = 30;
        b.max = 70;
        b.name = "Unknown";
        return  b;
    }

}
