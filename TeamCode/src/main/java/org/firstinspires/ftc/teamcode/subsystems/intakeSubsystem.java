package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem;

public class intakeSubsystem  extends Subsystem {

    states currentState = states.IDLE;
    states wantedState = states.IDLE;

    DcMotor intake;

    public intakeSubsystem(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
    }

    public enum states {
        IDLE,
        INTAKE,
        OUTAKE
    }

    public void setWantedState(states state) {this.wantedState = state;}

    private void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
               setWantedState(states.IDLE);
               break;
            case INTAKE:
                setWantedState(states.INTAKE);
                break;
            case OUTAKE:
                setWantedState(states.OUTAKE);
        }
    }

    private void handleIdleState() {
        intake.setPower(0);
    }

    private void handleIntakeState() {
        intake.setPower(-1);
    }

    private void handleOutakeState() {
        intake.setPower(1);
    }

    public void periodic() {
        handleStateTransitions();
        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;
            case INTAKE:
                handleIntakeState();
                break;
            case OUTAKE:
                handleOutakeState();
                break;
        }
    }
}
