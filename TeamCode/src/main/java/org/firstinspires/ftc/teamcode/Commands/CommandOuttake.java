package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandOuttake implements Command{
    IntakeState intakeStates;

    public CommandOuttake(IntakeState intakeStates){
        this.intakeStates = intakeStates;
    }

    @Override
    public void start() {
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
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return intakeStates;
    }
}
