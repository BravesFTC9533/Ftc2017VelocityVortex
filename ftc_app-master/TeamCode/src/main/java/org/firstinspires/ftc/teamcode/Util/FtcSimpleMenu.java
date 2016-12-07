package org.firstinspires.ftc.teamcode.Util;

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
    private ArrayList<Option> options = new ArrayList();

    public FtcSimpleMenu(String menuTitle, Telemetry telemetry, Gamepad gamepad) {
        this.menuTitle = menuTitle;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    public void addOption(String option, String[] choices) {
        this.options.add(new Option(option, choices));
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
                this.telemetry.addData(">> " + o2.getName(), (Object)o2.getCurrentChoice());
            } else {
                this.telemetry.addData(o2.getName(), (Object)o2.getCurrentChoice());
            }
            ++count;
        }
        this.telemetry.update();
    }

    private boolean checkButton(boolean b, int i) {
        if (b != this.buttonStates[i]) {
            this.buttonStates[i] = !this.buttonStates[i];
            return this.buttonStates[i];
        }
        return false;
    }
}