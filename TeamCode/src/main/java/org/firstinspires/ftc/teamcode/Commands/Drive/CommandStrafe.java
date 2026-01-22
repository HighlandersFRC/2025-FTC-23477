package org.firstinspires.ftc.teamcode.Commands.Drive;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandStrafe implements Command {
    DriveStates drive;
    double distance;
    public CommandStrafe (DriveStates drive, double distance) {
        this.drive = drive;
        this.distance = distance;
    }

    @Override
    public void start() {
        drive.driveForwardDriveDistanceY(distance);
        drive.setWantedState(DriveStates.DRIVE_STATE.DRIVE_STRAFE);
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
        return drive.isFinishedY();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}
