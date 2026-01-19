package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.json.JSONException;

public interface Command {

    void start();

    void execute() throws JSONException;

    void end();

    boolean isFinished();

    Subsystem getRequiredSubsystem();
}