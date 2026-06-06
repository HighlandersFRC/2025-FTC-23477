package org.firstinspires.ftc.teamcode;

public interface Command {

    void start();

    void execute();

    void end();

    boolean isFinished();

    Subsystem getRequiredSubsystem();
}