package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.rotateSubsystem;

public class commandClockwise implements Command {

    rotateSubsystem subsystem;

    public commandClockwise(HardwareMap hardwareMap) {
        subsystem = new rotateSubsystem("turn");
        subsystem.init(hardwareMap);
    }

    @Override
    public void start() {
        subsystem.setWantedState(rotateSubsystem.rotateStates.CLOCKWISE_TURN);
    }

    @Override
    public void execute() {
        subsystem.periodic();
    }

    @Override
    public void end() {
        subsystem.setWantedState(rotateSubsystem.rotateStates.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return subsystem;
    }
}
