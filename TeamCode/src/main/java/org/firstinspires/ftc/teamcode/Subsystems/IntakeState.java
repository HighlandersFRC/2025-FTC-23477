package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.IntakeHoldPID;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeState extends Subsystem {

    private INTAKE_STATE wantedSuperState = INTAKE_STATE.IDLE;
    private INTAKE_STATE currentSuperState = INTAKE_STATE.IDLE;
    private DcMotor IntakeMotor;
    public IntakeState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setWantedState(INTAKE_STATE intakeState){
        wantedSuperState = intakeState;
    }

    public enum INTAKE_STATE {
        DEFAULT,
        IDLE,
        OUTTAKE,
        INTAKE
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = INTAKE_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = INTAKE_STATE.IDLE;
                break;
            case OUTTAKE:
                currentSuperState = INTAKE_STATE.OUTTAKE;
                break;
            case INTAKE:
                currentSuperState = INTAKE_STATE.INTAKE;
                break;
        }
    }

    private void handleDefaultState() {
//        IntakeHoldPID.setSetPoint(IntakeMotor.getCurrentPosition());
//        IntakeHoldPID.updatePID(IntakeMotor.getCurrentPosition());;
//        IntakeMotor.setPower(-IntakeHoldPID.getResult());

        IntakeMotor.setPower(0);
    }

    private void handleIdleState() {

    }

    private void handleOuttakeState() {
        IntakeMotor.setPower(1);
    }

    private void handleIntakeState() {
        IntakeMotor.setPower(-1);
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
            case OUTTAKE:
                handleOuttakeState();
                break;
            case INTAKE:
                handleIntakeState();
                break;
        }
    }

}