package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandIndex implements Command{
    IndexerState indexerState;
    long startTime;
    long duration;

    public CommandIndex(IndexerState indexerState, long millis){
        this.indexerState = indexerState;
        this.duration = millis;
    }


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        indexerState.setWantedState(IndexerState.INDEXER_STATE.INDEX);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
       indexerState.setWantedState(IndexerState.INDEXER_STATE.IDLE);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return indexerState;
    }
}
