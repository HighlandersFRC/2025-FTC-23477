package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.function.BooleanSupplier;

public class WaitUntil implements Command {

    private final BooleanSupplier condition;
    private final Subsystem finalSubsytem;

    public WaitUntil(BooleanSupplier condition, Subsystem finalSubsytem) {
        this.condition = condition;
        this.finalSubsytem = finalSubsytem;
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        // just wait
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return finalSubsytem;
    }



}
