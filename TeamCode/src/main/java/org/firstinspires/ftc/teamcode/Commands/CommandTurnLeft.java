
package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandTurnLeft implements Command{
    DriveStates drive;
    double degrees;
    public CommandTurnLeft(DriveStates drive, double degrees) {
        this.drive = drive;
        this.degrees = degrees;
    }

    @Override
    public void start() {
        drive.driveTurnDriveDistanceTheta(degrees);
        drive.setWantedState(DriveStates.DRIVE_STATE.DRIVE_TURN_LEFT);
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
        return drive.isFinishedTheta();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}
