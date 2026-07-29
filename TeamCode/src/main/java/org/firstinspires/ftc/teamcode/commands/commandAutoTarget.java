package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.cameraSubsystem;

public class commandAutoTarget implements Command {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    cameraSubsystem camera;

    public commandAutoTarget(HardwareMap hardwaremap, Telemetry telemetry, cameraSubsystem subsystem) {this.hardwareMap = hardwaremap; this.telemetry = telemetry; this.camera = subsystem;}

    @Override
    public void start() {
        camera.init(hardwareMap, telemetry);
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
