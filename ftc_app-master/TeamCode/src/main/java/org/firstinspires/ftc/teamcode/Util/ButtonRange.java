package org.firstinspires.ftc.teamcode.Util;

/**
 * Created by TheSp on 1/14/2017.
 */

public class ButtonRange {

    private double offset_inchset;

    private String name;
    public double getOffset(){
        return offset_inchset;
    }



    public String getName() { return  name; }




    public static ButtonRange LeftButton() {
        ButtonRange b = new ButtonRange();
        b.offset_inchset = -3.5;
//        b.min = -105 -20;
//        b.max = -70 -20;
        b.name = "Left Button";
        return  b;

    }
    public static ButtonRange RightButton(){
        ButtonRange b = new ButtonRange();
        b.offset_inchset = 2;
//        b.min = 25 - 10;
//        b.max = 60 -10;
        b.name = "Right Button";
        return  b;
    }

    public static ButtonRange Unknown() {
        ButtonRange b = new ButtonRange();
       b.offset_inchset = 0;
        b.name = "Unknown";
        return  b;
    }

}
