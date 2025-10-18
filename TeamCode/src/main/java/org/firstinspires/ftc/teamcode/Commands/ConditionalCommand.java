package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import java.util.function.BooleanSupplier;

public class ConditionalCommand implements Command {
    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;
    private Command activeCommand;
    private boolean hasTriggered = false;

    public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
    }

    @Override
    public void start() {
        if (condition.getAsBoolean()) {
            activeCommand = onTrue;
            hasTriggered = true;
        } else {
            activeCommand = onFalse;
        }
        if (activeCommand != null) activeCommand.start();
    }

    @Override
    public void execute() {
        if (!hasTriggered && condition.getAsBoolean()) {
            if (activeCommand != null) activeCommand.end();
            activeCommand = onTrue;
            activeCommand.start();
            hasTriggered = true;
        }

        if (activeCommand != null) activeCommand.execute();
    }

    @Override
    public void end() {
        if (activeCommand != null) activeCommand.end();
    }

    @Override
    public boolean isFinished() {
        return activeCommand != null && activeCommand.isFinished();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
