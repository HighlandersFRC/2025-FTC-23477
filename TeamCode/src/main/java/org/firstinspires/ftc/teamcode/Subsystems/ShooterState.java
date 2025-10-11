package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterState extends Subsystem {

    private SHOOTER_STATE wantedSuperState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentSuperState = SHOOTER_STATE.IDLE;

    public ShooterState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {

    }

    public void setWantedState(SHOOTER_STATE shooterState){
        wantedSuperState = shooterState;
    }

    public enum SHOOTER_STATE {
        DEFAULT,
        IDLE
    }

    private SHOOTER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SHOOTER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = SHOOTER_STATE.IDLE;
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