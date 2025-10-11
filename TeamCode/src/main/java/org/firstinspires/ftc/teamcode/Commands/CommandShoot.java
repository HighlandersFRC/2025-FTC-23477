package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command{
    ShooterState shooterStates;

    public CommandShoot(ShooterState shooterStates) {
        this.shooterStates = shooterStates;
    }

    @Override
    public void start() {
        shooterStates.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
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
        return System.currentTimeMillis() == 2000;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooterStates;
    }
}
