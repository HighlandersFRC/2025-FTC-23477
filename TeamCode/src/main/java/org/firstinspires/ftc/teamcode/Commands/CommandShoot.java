package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooter;
    private final double distance;
    private final long timeoutMs;  // optional safety timeout
    private long startTime;

    public CommandShoot(ShooterState shooter, double distance, long timeoutMs) {
        this.shooter = shooter;
        this.distance = distance;
        this.timeoutMs = timeoutMs;
    }

    @Override
    public void start() {
        shooter.setTargetRPMFromDistance(distance);
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Shooter spins up automatically in periodic()
    }

    @Override
    public boolean isFinished() {
        // Wait until the flywheel is ready OR timeout expires
        return shooter.isAtTargetVelocityStable();
    }

    @Override
    public void end() {
        shooter.setWantedState(ShooterState.SHOOTER_STATE.IDLE);
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooter;
    }
}
