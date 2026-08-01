package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.driveSubsystem;

public class commandDriveBack implements Command {

    double drive_distance;
    HardwareMap hardwareMap;

    driveSubsystem subsystem = new driveSubsystem("subsystem");

    public commandDriveBack(double distance_inches, HardwareMap hardwareMap) {
        this.drive_distance = distance_inches;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void start() {
      subsystem.init(hardwareMap);
      subsystem.setBackwardDistance(drive_distance);
      subsystem.setWantedState(driveSubsystem.rotateStates.DRIVE_BACKWARD_INCHES);
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
        return subsystem.isFinishedBackward();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return subsystem;
    }
}
