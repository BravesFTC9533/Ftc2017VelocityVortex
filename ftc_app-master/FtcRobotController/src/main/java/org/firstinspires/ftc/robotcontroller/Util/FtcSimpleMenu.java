package org.firstinspires.ftc.robotcontroller.Util;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.Telemetry;



class Option
{
    private String name;
    private String[] choices;
    public int choiceIndex = 0;

    public Option(String name, String[] choices)
    {
        this.name = name;
        this.choices = choices;
    }
    public Option(String name, double max, double min, double inc)
    {
        this.name = name;

        int size = (int) ((max-min)/inc);
        choices = new String[size];
        for (int i = 0; i < size; i++)
        {
            choices[i] = String.valueOf(inc*i);
        }

        choiceIndex = 0;           //fix this rock hard code
    }

    public String getName()
    {
        return name;
    }
    public String[] getChoices()
    {
        return choices;
    }
    public String getCurrentChoice()
    {
        return choices[choiceIndex];
    }
    public int getNumChoices()
    {
        return choices.length;
    }
}

public class FtcSimpleMenu {
    private Telemetry telemetry;
    private String menuTitle;
    private Gamepad gamepad;
    int currentOption = 0;
    private boolean[] buttonStates = new boolean[4];
    private ArrayList<Option> options;

    public FtcSimpleMenu(String menuTitle) {
        this.menuTitle = menuTitle;
        options = new ArrayList();
    }


    public void setTelemetry(Telemetry t)
    {
        this.telemetry = t;
    }
    public void setGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }

    public void clearOptions()
    {
        options.clear();
    }
    public void loadFrom(ArrayList<Option>options)
    {
        this.options = options;
    }
    public ArrayList<Option> getOptionsConfig()
    {
        return options;
    }

    public void addOption(String option, String[] choices) {
        this.options.add(new Option(option, choices));
    }
    public void addOption(String option, double max, double min, double inc)
    {
        this.options.add(new Option(option, max, min, inc));
    }

    public String getCurrentChoiceOf(String option) {
        for (Option o : this.options) {
            if (!o.getName().equals(option)) continue;
            return o.getCurrentChoice();
        }
        return "ERROR - NO OPTION BY THAT NAME";
    }

    public void displayMenu() {
        Option o;
        if (this.checkButton(this.gamepad.dpad_down, 1)) {
            this.currentOption = this.currentOption < this.options.size() - 1 ? ++this.currentOption : 0;
        } else if (this.checkButton(this.gamepad.dpad_up, 0)) {
            this.currentOption = this.currentOption > 0 ? --this.currentOption : this.options.size() - 1;
        } else if (this.checkButton(this.gamepad.dpad_left || this.gamepad.b, 2)) {
            o = this.options.get(this.currentOption);
            o.choiceIndex = o.choiceIndex > 0 ? --o.choiceIndex : o.getNumChoices() - 1;
        } else if (this.checkButton(this.gamepad.dpad_right || this.gamepad.a, 3)) {
            o = this.options.get(this.currentOption);
            o.choiceIndex = o.choiceIndex < o.getNumChoices() - 1 ? ++o.choiceIndex : 0;
        }
        this.telemetry.addData("Menu", (Object)this.menuTitle);
        int count = 0;
        for (Option o2 : this.options) {
            if (this.currentOption == count) {
                this.telemetry.addData(">> " + o2.getName(), o2.getCurrentChoice());
            } else {
                this.telemetry.addData(o2.getName(), o2.getCurrentChoice());
            }
            ++count;
        }
        this.telemetry.update();
    }

    public void displayConfig(Telemetry telemetry)
    {
        for (Option o2 : this.options)
        {
            telemetry.addData(o2.getName(), o2.getCurrentChoice());
            telemetry.update();
        }
    }

    private boolean checkButton(boolean b, int i) {
        if (b != this.buttonStates[i]) {
            this.buttonStates[i] = !this.buttonStates[i];
            return this.buttonStates[i];
        }
        return false;
    }
}