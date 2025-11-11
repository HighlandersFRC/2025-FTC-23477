package org.firstinspires.ftc.teamcode.Commands;

public interface Command {

    void start();

    boolean execute();

    void end();

    boolean isFinished();

    boolean getRequiredSubsystem();
}