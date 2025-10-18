package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagState extends Subsystem {

    private APRIL_TAG_STATE wantedSuperState = APRIL_TAG_STATE.IDLE;
    private APRIL_TAG_STATE currentSuperState = APRIL_TAG_STATE.IDLE;

    public AprilTagState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {

    }
    public void setWantedState(APRIL_TAG_STATE aprilTagState){
        wantedSuperState = aprilTagState;
    }

    public enum APRIL_TAG_STATE {
        DEFAULT,
        IDLE,
        RANGE_TRACK
    }

    private APRIL_TAG_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = APRIL_TAG_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = APRIL_TAG_STATE.IDLE;
                break;
            case RANGE_TRACK:
                currentSuperState = APRIL_TAG_STATE.RANGE_TRACK;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {

    }

    private void handleIdleState() {

    }

    private void handleRangeTrackingState() {

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
            case RANGE_TRACK:
                handleRangeTrackingState();
                break;
        }
    }

}