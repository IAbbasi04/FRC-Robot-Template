package lib.hardware.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandMayflashGamepad extends CommandGenericHID {
    public CommandMayflashGamepad(int port) {
        super(port);
    }

    // TODO - These are all wrong so eventually fix them
    // MOVE THIS INTO A MAYFLASH GAMEPAD CLASS THAT CONTAINS AN ENUM FOR ALL BUTTON POSITIONS
    // AND USE THIS CLASS FOR ACCESSING IT 

    public Trigger L1() {
        return super.button(1);
    }

    public Trigger L2() {
        return super.button(2);
    }

    public Trigger L3() {
        return super.button(3);
    }

    public Trigger L4() {
        return super.button(4);
    }

    public Trigger R1() {
        return super.button(5);
    }

    public Trigger R2() {
        return super.button(6);
    }

    public Trigger R3() {
        return super.button(7);
    }

    public Trigger R4() {
        return super.button(8);
    }

    public Trigger T1() {
        return super.button(9);
    }

    public Trigger T2() {
        return super.button(10);
    }

    public Trigger T3() {
        return super.button(11);
    }

    public Trigger T4() {
        return super.button(12);
    }
}