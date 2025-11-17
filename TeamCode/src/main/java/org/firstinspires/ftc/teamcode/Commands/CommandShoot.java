package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooterState;
    private final double targetRPM;
    private final long shootDuration; // Total time to keep shooting
    private long startTime;

    /**
     * @param shooterState The shooter subsystem
     * @param targetRPM Target velocity for the shooter
     * @param shootDuration Total duration to keep the shooter running (milliseconds)
     */
    public CommandShoot(ShooterState shooterState, double targetRPM, long shootDuration) {
        this.shooterState = shooterState;
        this.targetRPM = targetRPM;
        this.shootDuration = shootDuration;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        shooterState.setTargetRPM(targetRPM);
        shooterState.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public void execute() {
        // Nothing needed here - just maintain shooter state
    }

    @Override
    public void end() {
        shooterState.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= shootDuration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooterState;
    }

    public boolean isAtTargetVelocity() {
        return shooterState.isAtTargetVelocity();
    }
}