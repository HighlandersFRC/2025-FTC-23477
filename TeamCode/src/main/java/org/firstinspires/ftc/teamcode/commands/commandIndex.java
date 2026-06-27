package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.indexerSubsystem;

public class commandIndex implements Command {

    HardwareMap hardwaremap;
    long duration;
    indexerSubsystem subsystem = new indexerSubsystem("subsystem");

    public commandIndex(HardwareMap hardwareMap, long time) {
        this.hardwaremap = hardwareMap;
        this.duration = time;
    }

    @Override
    public void start() {
        subsystem.init(hardwaremap);
        subsystem.setIndexDuration(duration);
        subsystem.setWantedState(indexerSubsystem.states.INDEXING);
    }

    @Override
    public void execute() {
        subsystem.periodic();
    }

    @Override
    public void end() {
        subsystem.setWantedState(indexerSubsystem.states.IDLE);
    }

    @Override
    public boolean isFinished() {
        return subsystem.isFinished();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return subsystem;
    }
}
