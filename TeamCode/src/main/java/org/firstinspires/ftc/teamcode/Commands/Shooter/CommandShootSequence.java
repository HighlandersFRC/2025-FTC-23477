package org.firstinspires.ftc.teamcode.Commands.Shooter;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Queuer.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandShootSequence implements Command {

    private final ShooterState shooter;
    private final SequencerState sequencer;
    private final CommandScheduler scheduler;
    private final double rpm;
    private final long totalDuration;

    private long startTime;
    private boolean sequencerStarted = false;


    public CommandShootSequence(CommandScheduler scheduler,
                                ShooterState shooter,
                                SequencerState sequencer,
                                double rpm,
                                long duration
                                ) {
        this.scheduler = scheduler;
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.rpm = rpm;
        this.totalDuration = duration;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        shooter.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
    }

    @Override
    public void execute() {

        // Wait until shooter is at speed
        if (!sequencerStarted && shooter.isAtTargetVelocity()) {
            // Run sequencer in parallel
            scheduler.schedule(
                    new CommandSpinRight(sequencer, totalDuration)
            );
            sequencerStarted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= totalDuration;
    }

    @Override
    public void end() {
        shooter.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return shooter;
    }
}
