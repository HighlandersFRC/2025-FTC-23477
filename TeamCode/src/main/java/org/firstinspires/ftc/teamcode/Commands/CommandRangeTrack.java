package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
public class CommandRangeTrack implements Command{
    AprilTagState aprilTagState;


    public CommandRangeTrack(AprilTagState aprilTagState){
        this.aprilTagState = aprilTagState;
    }

    @Override
    public void start() {
        aprilTagState.setWantedState(AprilTagState.APRIL_TAG_STATE.RANGE_TRACK);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        aprilTagState.setWantedState(AprilTagState.APRIL_TAG_STATE.DEFAULT);
    }   

    @Override
    public boolean isFinished() {
        return true; // Fix this asap
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return aprilTagState;
    }
}
