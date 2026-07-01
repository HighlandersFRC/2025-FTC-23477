package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.indexerSubsystem;

public class commandIndex implements Command {
    private final HardwareMap hardwareMap;
    private final double durationSeconds;
    private final indexerSubsystem subsystem = new indexerSubsystem("indexer");

    public commandIndex(HardwareMap hardwareMap, double timeSeconds) {
        this.hardwareMap = hardwareMap;
        durationSeconds = timeSeconds;
    }

    @Override
    public void start() {
        subsystem.init(hardwareMap);
        subsystem.setIndexDuration(durationSeconds);
        subsystem.setWantedState(indexerSubsystem.states.INDEXING);
    }

    @Override
    public void execute() {
        subsystem.periodic();
    }

    @Override
    public void end() {
        subsystem.setWantedState(indexerSubsystem.states.IDLE);
        subsystem.periodic();
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
