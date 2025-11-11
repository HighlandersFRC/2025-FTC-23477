package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;

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
    public boolean execute() {

        return false;
    }

    @Override
    public void end() {
        aprilTagState.setWantedState(AprilTagState.APRIL_TAG_STATE.RANGE_TRACK);
    }   

    @Override
    public boolean isFinished() {
        return true; // Fix this asap
    }

    @Override
    public boolean getRequiredSubsystem() {
        return aprilTagState;
    }
}
