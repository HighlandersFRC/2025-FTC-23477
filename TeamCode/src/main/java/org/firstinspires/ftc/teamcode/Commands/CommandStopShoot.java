package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandStopShoot implements Command{
    ShooterState shooterStates;

    public CommandStopShoot(ShooterState shooterStates) {
        this.shooterStates = shooterStates;
    }

    @Override
    public void start() {
        shooterStates.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        shooterStates.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooterStates;
    }


}
