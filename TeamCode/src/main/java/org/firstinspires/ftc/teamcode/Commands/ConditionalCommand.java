package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.function.BooleanSupplier;

public class ConditionalCommand implements Command {

    private final Command onTrue;
    private final Command onFalse;
    private final BooleanSupplier condition;

    private Command activeCommand;
    private boolean hasSwitchedToTrue = false;

    public ConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
    }

    @Override
    public void start() {
        // Always start in onFalse unless condition is already true
        if (condition.getAsBoolean()) {
            activeCommand = onTrue;
            hasSwitchedToTrue = true;
        } else {
            activeCommand = onFalse;
        }

        if (activeCommand != null) {
            activeCommand.start();
        }
    }

    @Override
    public void execute() {
        // One-way switch: false → true
        if (!hasSwitchedToTrue && condition.getAsBoolean()) {
            if (activeCommand != null) {
                activeCommand.end();
            }

            activeCommand = onTrue;
            activeCommand.start();
            hasSwitchedToTrue = true;
        }

        if (activeCommand != null) {
            activeCommand.execute();
        }
    }

    @Override
    public void end() {
        if (activeCommand != null) {
            activeCommand.end();
        }
    }

    @Override
    public boolean isFinished() {
        // ❗ CRITICAL FIX:
        // Do NOT finish until we've switched to onTrue
        if (!hasSwitchedToTrue) {
            return false;
        }

        return activeCommand != null && activeCommand.isFinished();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
