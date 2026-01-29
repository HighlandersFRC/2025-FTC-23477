package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShoot implements Command {

    private final ShooterState shooter;
    private final double distance;
    private long startTime;
    private long duration;

    public CommandShoot(ShooterState shooter, double distance, long duration) {
        this.shooter = shooter;
        this.distance = distance;
        this.duration = duration;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        shooter.setTargetRPMFromDistance(distance);
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
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
