package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooter;
    private final double distance;
    private final long durationMs;

    private long atSpeedStartTime = -1; // timer hasn't started yet

    public CommandShoot(ShooterState shooter, double distance, long durationMs) {
        this.shooter = shooter;
        this.distance = distance;
        this.durationMs = durationMs;
    }

    @Override
    public void start() {
        atSpeedStartTime = -1; // reset timer
        shooter.setTargetRPMFromDistance(distance);
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public void execute() {
        // Shooter PID runs in ShooterState.periodic()

        // Start timer only once, when at target velocity
        if (shooter.isAtTargetVelocity() && atSpeedStartTime < 0) {
            atSpeedStartTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        // Timer hasn't started yet → keep running
        if (atSpeedStartTime < 0) {
            return false;
        }

        // End after holding speed for durationMs
        return System.currentTimeMillis() - atSpeedStartTime >= durationMs;
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
