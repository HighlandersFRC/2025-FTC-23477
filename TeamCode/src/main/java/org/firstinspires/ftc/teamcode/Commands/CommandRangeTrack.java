package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandRangeTrack implements Command {

    AprilTagState aprilTagState;
    private long startTime;
    private long timeoutMs = 3000; // 3-second timeout

    public CommandRangeTrack(AprilTagState aprilTagState){
        this.aprilTagState = aprilTagState;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        aprilTagState.setWantedState(AprilTagState.TAG_STATE.RANGE_TRACK);
    }

    @Override
    public void execute() {
        // Tag logic handled inside subsystem.periodic()
    }

    @Override
    public void end() {
        aprilTagState.setWantedState(AprilTagState.TAG_STATE.IDLE);
    }

    @Override
    public boolean isFinished() {
        boolean timeout = (System.currentTimeMillis() - startTime) > timeoutMs;
        return aprilTagState.isAtTarget() || timeout;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return aprilTagState;
    }
}
