package org.firstinspires.ftc.teamcode.Commands;





import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class CommandQueue implements Command{
    QueueState queueState;
    long startTime;
    long duration;

    public CommandQueue(QueueState queueState, long millis){
        this.queueState = queueState;
        this.duration = millis;
    }


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        queueState.setWantedState(QueueState.QUEUE_STATE.QUEUE);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end() {
        queueState.setWantedState(QueueState.QUEUE_STATE.IDLE);
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return queueState;
    }
}
