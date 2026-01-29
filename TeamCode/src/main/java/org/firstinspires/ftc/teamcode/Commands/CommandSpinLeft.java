package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandSpinLeft implements Command {
    SequencerState sequencerState;
    long startTime;
    long duration;
    public CommandSpinLeft(SequencerState sequencerState, long millis){
        this.sequencerState = sequencerState;
        this.duration = millis;
    }


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.SPIN_LEFT);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return sequencerState;
    }


}
