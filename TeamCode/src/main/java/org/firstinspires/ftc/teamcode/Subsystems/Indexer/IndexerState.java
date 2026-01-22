package org.firstinspires.ftc.teamcode.Subsystems.Indexer;



import static org.firstinspires.ftc.teamcode.Tools.Constants.INDEXER_PID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.INDEXER_POS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class IndexerState extends Subsystem {
    private INDEXER_STATE wantedSuperState = INDEXER_STATE.IDLE;
    private INDEXER_STATE currentSuperState = INDEXER_STATE.IDLE;
    DcMotor Indexer;

    public IndexerState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        Indexer = hardwareMap.dcMotor.get("IndexerMotor");

        INDEXER_PID.setMaxOutput(1);
        INDEXER_PID.setMinOutput(-1);
    }

    public void setWantedState(INDEXER_STATE indexerState){
        wantedSuperState = indexerState;
    }

    public enum INDEXER_STATE{
        DEFAULT,
        IDLE,
        INDEX,
        REMOVE,
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = INDEXER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = INDEXER_STATE.IDLE;
                break;
            case INDEX:
                currentSuperState = INDEXER_STATE.INDEX;
                break;
            case REMOVE:
                currentSuperState = INDEXER_STATE.REMOVE;
                break;
        }
    }

    private void handleDefaultState() {
        Indexer.setPower(0);
    }

    private void handleIdleState() {
        setPosition(Indexer.getCurrentPosition());
        runToPosition();
    }

    private void handleIndexState() {
        Indexer.setPower(1);
    }

    private void handleRemoveState() {
        Indexer.setPower(-1);
    }

    private void setPosition(double position) {
        INDEXER_POS = position;
    }

    private void runToPosition() {
        double currentPos = Indexer.getCurrentPosition();
        INDEXER_PID.setSetPoint(INDEXER_POS);
        INDEXER_PID.updatePID(currentPos);
        Indexer.setPower(INDEXER_PID.getResult());

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
            case INDEX:
                handleIndexState();
                break;
            case REMOVE:
                handleRemoveState();
                break;
        }
    }

}