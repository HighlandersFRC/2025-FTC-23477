package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandTurnRightTime implements Command {
    DriveStates driveStates;
    long startTime;
    long duration;

    public CommandTurnRightTime(DriveStates driveStates, long millis){
        this.driveStates = driveStates;
        this.duration = millis;
    }


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        driveStates.setWantedState(DriveStates.DRIVE_STATE.DRIVE_TURN_RIGHT_TIME);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        driveStates.setWantedState(DriveStates.DRIVE_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return driveStates;
    }



}
