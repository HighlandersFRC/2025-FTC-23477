package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooterState;
    private final double targetRPM;
    private final long duration;
    private long startTime;
    public CommandShoot(ShooterState shooterState, double targetRPM, long duration) {
        this.shooterState = shooterState;
        this.targetRPM = targetRPM;
        this.duration = duration;

    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        shooterState.setTargetRPM(targetRPM);
        shooterState.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public void execute() {

    }


    @Override
    public void end() {
        shooterState.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooterState;
    }
}
