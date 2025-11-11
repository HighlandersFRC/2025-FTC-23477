package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

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
    public boolean execute() {

        return false;
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
    public boolean getRequiredSubsystem() {
        return shooterStates;
    }
}
