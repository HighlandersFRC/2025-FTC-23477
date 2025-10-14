package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {
    ShooterState shooterStates;
    long startTime;
    long duration;
    public CommandShoot(ShooterState shooterStates, long seconds) {
        this.shooterStates = shooterStates;
        this.duration = seconds * 1000;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
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
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooterStates;
    }
}
