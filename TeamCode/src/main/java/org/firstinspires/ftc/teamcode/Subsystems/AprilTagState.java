package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class AprilTagState extends Subsystem {

    private APRIL_TAG_STATE wantedSuperState = APRIL_TAG_STATE.IDLE;
    private APRIL_TAG_STATE currentSuperState = APRIL_TAG_STATE.IDLE;

    private Limelight3A limelight;
    private double lastTx = 0;


    private final PID forwardPID = new PID(1.0, 0.0, 0.01);
    private final PID strafePID  = new PID(1.0, 0.0, 0.01);
    private final PID turnPID    = new PID(0.045, 0.0, 0.030);

    private Drive drive;


    private double desiredDistance = 1;
    private double desiredX = 0.0;
    private double desiredTx = 0.0;

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