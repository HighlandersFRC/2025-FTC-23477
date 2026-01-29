package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandTurnRightTime implements Command {
    DriveStates driveStates;
    double degrees;

    public CommandTurnRightTime(DriveStates driveStates, double degrees) {
        this.driveStates = driveStates;
        this.degrees = degrees;
    }

    @Override
    public void start() {
        driveStates.driveTurnDriveDistanceThetaIMU(degrees);
        driveStates.setWantedState(DriveStates.DRIVE_STATE.DRIVE_TURN_RIGHT_TIME);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return driveStates.isFinishedThetaIMU();
    }

    @Override
    public void end() {
        driveStates.setWantedState(DriveStates.DRIVE_STATE.DEFAULT);
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return driveStates;
    }
}
