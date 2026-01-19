package org.firstinspires.ftc.teamcode.Commands;


import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooter;
    private final double distance;
    private final long durationMs;
    private long secondsPassed;


    public CommandShoot(ShooterState shooter, double distance, long durationMs) {
        this.shooter = shooter;
        this.distance = distance;
        this.durationMs = durationMs;
    }

    @Override
    public void start() {
        shooter.setTargetRPMFromDistance(distance);
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
        secondsPassed = System.currentTimeMillis();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - secondsPassed >= durationMs;
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
