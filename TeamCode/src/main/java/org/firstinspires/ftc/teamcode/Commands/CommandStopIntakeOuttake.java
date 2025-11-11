package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;

public class CommandStopIntakeOuttake implements Command{
    IntakeState intakeStates;

    public CommandStopIntakeOuttake(IntakeState intakeStates){
        this.intakeStates = intakeStates;
    }

    @Override
    public void start() {
        intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
    }

    @Override
    public boolean execute() {

        return false;
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
    public boolean getRequiredSubsystem() {
        return intakeStates;
    }
}
