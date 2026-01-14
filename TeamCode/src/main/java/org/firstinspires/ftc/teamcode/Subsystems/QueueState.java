package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class QueueState extends Subsystem {
    private QUEUE_STATE wantedSuperState = QUEUE_STATE.IDLE;
    private QUEUE_STATE currentSuperState = QUEUE_STATE.IDLE;
    DcMotor Queuer;

    public QueueState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        Queuer = hardwareMap.dcMotor.get("QueueMotor");
        Queuer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setWantedState(QUEUE_STATE queueState){
        wantedSuperState = queueState;
    }

    public enum QUEUE_STATE{
        DEFAULT,
        IDLE,
        QUEUE
    }

    private QUEUE_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = QUEUE_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = QUEUE_STATE.IDLE;
                break;
            case QUEUE:
                currentSuperState = QUEUE_STATE.QUEUE;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
    }

    private void handleIdleState() {
        Queuer.setPower(0);
    }

    private void handleQueueState() {
        Queuer.setPower(1);
    }




    @Override
    public void periodic() {
        handleStateTransitions();
        switch (currentSuperState) {
            case DEFAULT:
                handleDefaultState();
                break;
            case IDLE:
                handleIdleState();
                break;
        }
    }

}