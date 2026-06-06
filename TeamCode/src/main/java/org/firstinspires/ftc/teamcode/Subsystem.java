package org.firstinspires.ftc.teamcode;

public abstract class Subsystem {
    private String name;
    private Command defaultCommand;

    public Subsystem(String name) {
        this.name = name;
    }

    public Subsystem() {

    }

    public String getName() {
        return name;
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void runDefaultCommand() {
        if (defaultCommand != null) {
            defaultCommand.execute();
        }
    }

    public void periodic(){};
}