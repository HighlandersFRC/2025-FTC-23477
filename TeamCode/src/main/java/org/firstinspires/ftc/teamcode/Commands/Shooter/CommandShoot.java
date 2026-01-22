package org.firstinspires.ftc.teamcode.Commands.Shooter;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooter;
    private final double distance;

    public CommandShoot(ShooterState shooter, double distance) {
        this.shooter = shooter;
        this.distance = distance;
    }

    @Override
    public void start() {
        shooter.setTargetRPMFromDistance(distance);
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
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
