package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeState extends Subsystem {

    private INTAKE_STATE wantedSuperState = INTAKE_STATE.IDLE;
    private INTAKE_STATE currentSuperState = INTAKE_STATE.IDLE;

    public IntakeState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {

    }

    public void setWantedState(INTAKE_STATE intakeState){
        wantedSuperState = intakeState;
    }

    public enum INTAKE_STATE {
        DEFAULT,
        IDLE
    }

    private INTAKE_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = INTAKE_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = INTAKE_STATE.IDLE;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {

    }

    private void handleIdleState() {

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