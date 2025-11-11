package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

public class CommandShoot implements Command {

    private final ShooterState shooterState;
    private final double targetRPM;
    private final double targetTicks;
    private final long duration;
    private long startTime;
    private final boolean useTickEnd;
    private boolean readyToFeed = false;

    public CommandShoot(ShooterState shooterState, double targetRPM, double targetTicks, long duration, boolean useTickEnd) {
        this.shooterState = shooterState;
        this.targetRPM = targetRPM;
        this.targetTicks = targetTicks;
        this.duration = duration;
        this.useTickEnd = useTickEnd;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        shooterState.setTargetRPM(targetRPM);
        shooterState.setTargetTicks(targetTicks);
        shooterState.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public boolean execute() {
        if (shooterState.isAtTargetVelocity()) readyToFeed = true;
        return false;
    }

    public boolean readyToFeed() {
        return readyToFeed;
    }

    @Override
    public void end() {
        shooterState.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        if (useTickEnd && shooterState.hasReachedTargetTicks()) return true;
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public boolean getRequiredSubsystem() {
        return shooterState;
    }
}
