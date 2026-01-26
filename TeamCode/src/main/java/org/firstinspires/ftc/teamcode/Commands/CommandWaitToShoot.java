package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandWaitToShoot implements Command {

    private final ShooterState shooter;
    private final double distance;

    public CommandWaitToShoot(ShooterState shooter, double distance) {
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

    }

    @Override
    public boolean isFinished() {
        return shooter.isAtTargetVelocity();
    }

    @Override
    public void end() {
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooter;
    }


}
