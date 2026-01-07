package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class SequencerState extends Subsystem {
    private SEQUENCER_STATE wantedSuperState = SEQUENCER_STATE.IDLE;
    private SEQUENCER_STATE currentSuperState = SEQUENCER_STATE.IDLE;
    CRServo sequencer;
    public SequencerState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        sequencer = hardwareMap.crservo.get("sequencer");
    }

    public void setWantedState(SEQUENCER_STATE sequencerState){
        wantedSuperState = sequencerState;
    }

    public enum SEQUENCER_STATE {
        DEFAULT,
        IDLE,
        SPIN_RIGHT,
        SPIN_LEFT
    }

    private SEQUENCER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SEQUENCER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = SEQUENCER_STATE.IDLE;
                break;
            case SPIN_LEFT:
                currentSuperState = SEQUENCER_STATE.SPIN_LEFT;
                break;
            case SPIN_RIGHT:
                currentSuperState = SEQUENCER_STATE.SPIN_RIGHT;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        sequencer.setPower(0);
    }

    private void handleIdleState() {

    }

    private void handleSpinRightState() {
        sequencer.setPower(1);
    }

    private void handleSpinLeftState() {
        sequencer.setPower(-1);
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
            case SPIN_RIGHT:
                handleSpinRightState();
                break;
            case SPIN_LEFT:
                handleSpinLeftState();
                break;
        }
    }

}