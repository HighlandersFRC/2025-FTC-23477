package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameraSubsystem;

public class commandAutoTarget implements Command {

    HardwareMap hardwareMap;

    cameraSubsystem camera = new cameraSubsystem("camera");

    public commandAutoTarget(HardwareMap hardwaremap) {this.hardwareMap = hardwaremap;}

    @Override
    public void start() {
        camera.init(hardwareMap);
        camera.setWantedState(cameraSubsystem.states.AUTO_TARGET);
    }

    @Override
    public void execute() {
        camera.periodic();
    }

    @Override
    public void end() {
        camera.setWantedState(cameraSubsystem.states.IDLE);
    }

    @Override
    public boolean isFinished() {
        return camera.isFinished();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return camera;
    }
}
