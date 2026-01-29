package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandFeedWhenAtVelocity implements Command {
    private final ShooterState shooter;
    private final QueueState queue;
    private boolean hasFed = false;

    public CommandFeedWhenAtVelocity(ShooterState shooter, QueueState queue) {
        this.shooter = shooter;
        this.queue = queue;
    }

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        if (!hasFed && shooter.isAtTargetVelocity()) {
            queue.setWantedState(QueueState.QUEUE_STATE.QUEUE); // start feeding
            hasFed = true;
        }
    }

    @Override
    public void end() {
queue.setWantedState(QueueState.QUEUE_STATE.IDLE);
    }



    @Override
    public boolean isFinished() {
        return hasFed; // auto-finish after feeding once
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
