package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;

public class commandShoot implements Command {
    private final HardwareMap hardwareMap;
    private final double durationSeconds;
    private final shooterSubsystem subsystem = new shooterSubsystem("shooter");

    public commandShoot(HardwareMap hardwareMap, double timeSeconds) {
        this.hardwareMap = hardwareMap;
        durationSeconds = timeSeconds;
    }

    @Override
    public void start() {
        subsystem.init(hardwareMap);
        subsystem.setShootingDuration(durationSeconds);
        subsystem.setWantedState(shooterSubsystem.shooter_states.SHOOTING);
    }

    @Override
    public void execute() {
        subsystem.periodic();
    }

    @Override
    public void end() {
        subsystem.setWantedState(shooterSubsystem.shooter_states.IDLE);
        subsystem.periodic();
    }

    @Override
    public boolean isFinished() {
        return subsystem.isFinishedShooting();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return subsystem;
    }
}
