package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem;

public class commandClockwise implements Command {

    driveSubsystem subsystem;

    public commandClockwise(HardwareMap hardwareMap) {
        subsystem = new driveSubsystem("turn");
        subsystem.init(hardwareMap);
    }

    @Override
    public void start() {
        subsystem.setWantedState(driveSubsystem.rotateStates.CLOCKWISE_TURN);
    }

    @Override
    public void execute() {
        subsystem.periodic();
    }

    @Override
    public void end() {
        subsystem.setWantedState(driveSubsystem.rotateStates.IDLE);
    }

    @Override
    public boolean isFinished() {
        return subsystem.isFinishedClockwise();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return subsystem;
    }
}
