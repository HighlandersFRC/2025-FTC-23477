package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooterSubsystem;

public class commandShoot implements Command {

    HardwareMap hardwaremap;
    long duration;
    shooterSubsystem subsystem = new shooterSubsystem("subsystem");

    public commandShoot(HardwareMap hardwareMap, long duration) {
        this.hardwaremap = hardwareMap;
        this.duration = duration;
    }

    @Override
    public void start() {
       subsystem.init(hardwaremap);
       subsystem.setShootingDuration(duration);
       subsystem.setWantedState(shooterSubsystem.shooter_states.SHOOTING);
    }

    @Override
    public void execute() {
        subsystem.periodic();
    }

    @Override
    public void end() {
        subsystem.setWantedState(shooterSubsystem.shooter_states.IDLE);
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
