package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterState extends Subsystem {

    private SHOOTER_STATE wantedSuperState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentSuperState = SHOOTER_STATE.IDLE;
    private DcMotor ShooterMotor;

    public ShooterState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setWantedState(SHOOTER_STATE shooterState){
        wantedSuperState = shooterState;
    }

    public enum SHOOTER_STATE {
        DEFAULT,
        IDLE,
        SHOOT,
        JAMMED
    }

    private SHOOTER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SHOOTER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = SHOOTER_STATE.IDLE;
                break;
            case SHOOT:
                currentSuperState = SHOOTER_STATE.SHOOT;
                break;
            case JAMMED:
                currentSuperState = SHOOTER_STATE.JAMMED;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        ShooterMotor.setPower(0);
    }

    private void handleIdleState() {

    }

    private void handleShootingState() {
        ShooterMotor.setPower(0.70);
    }

    private void handleJammedState() {
        ShooterMotor.setPower(-0.267);
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
            case SHOOT:
                handleShootingState();
                break;
            case JAMMED:
                handleJammedState();
                break;
        }
    }

}