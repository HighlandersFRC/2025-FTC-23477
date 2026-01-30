package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.INTAKE_PID_HOLD;
import static org.firstinspires.ftc.teamcode.Tools.Constants.INTAKE_POS;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class IntakeState extends Subsystem {

    private INTAKE_STATE wantedSuperState = INTAKE_STATE.IDLE;
    private INTAKE_STATE currentSuperState = INTAKE_STATE.IDLE;
    public DcMotor IntakeMotor;
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
//        INTAKE_PID_HOLD.setSetPoint(IntakeMotor.getCurrentPosition());
//        INTAKE_PID_HOLD.updatePID(IntakeMotor.getCurrentPosition());;
//        IntakeMotor.setPower(-INTAKE_PID_HOLD.getResult());

        IntakeMotor.setPower(0);
    }

    private void handleIdleState() {
        setPosition(IntakeMotor.getCurrentPosition());
        runToPosition();
    }

    private void handleOuttakeState() {
        IntakeMotor.setPower(1);
    }

    private void handleIntakeState() {
        IntakeMotor.setPower(-1);
    }

    private void setPosition(double position) {
        INTAKE_POS = position;
    }

    private void runToPosition() {
        double currentPos = IntakeMotor.getCurrentPosition();
        INTAKE_PID_HOLD.setSetPoint(INTAKE_POS);
        INTAKE_PID_HOLD.updatePID(currentPos);
        IntakeMotor.setPower(INTAKE_PID_HOLD.getResult());

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