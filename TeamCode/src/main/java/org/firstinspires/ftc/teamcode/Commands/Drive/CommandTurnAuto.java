package org.firstinspires.ftc.teamcode.Commands.Drive;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandTurnAuto implements Command {
    DriveStates drive;
    public CommandTurnAuto(DriveStates drive) {
        this.drive = drive;
    }

    @Override
    public void start() {
        drive.setWantedState(DriveStates.DRIVE_STATE.AUTO_TURN);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        drive.setWantedState(DriveStates.DRIVE_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return drive.isFinishedAutoTurnTheta();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}
