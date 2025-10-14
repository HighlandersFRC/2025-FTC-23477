package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Wait implements Command {
    private final long waitTime;
    private long startTime;
    private boolean isStarted;
    private double elapsed;

    public Wait(long waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public void start() {
        RobotLog.d("Wait Command Started: " + waitTime + "ms");
        startTime = System.currentTimeMillis();
        isStarted = true;
    }

    @Override
    public void execute() {
        if (isStarted) {
            elapsed = System.currentTimeMillis() - startTime;
            RobotLog.d("Wait Command Executing: WaitTime=" + waitTime + "ms, Elapsed=" + elapsed + "ms");
        }
    }

    @Override
    public void end() {
        isStarted = false;
        RobotLog.d("Wait Command Ended after " + waitTime + "ms");
    }

    @Override
    public boolean isFinished() {
        return elapsed >= waitTime;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}