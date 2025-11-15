package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandOuttake implements Command{
    IntakeState intakeStates;
    long startTime;
    long duration;

    public CommandOuttake(IntakeState intakeStates, long millis){
        this.intakeStates = intakeStates;
        this.duration = millis;
    }


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeStates;
    }
}