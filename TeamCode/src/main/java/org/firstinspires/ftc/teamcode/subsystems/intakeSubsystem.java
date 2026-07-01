package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem;

public class intakeSubsystem extends Subsystem {
    private states currentState = states.IDLE;
    private states wantedState = states.IDLE;

    private DcMotor intake;

    public intakeSubsystem(String name) {
        super(name);
    }

    public enum states {
        IDLE,
        INTAKE,
        OUTAKE
    }

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setWantedState(states state) {
        wantedState = state;
    }

    public void periodic() {
        currentState = wantedState;

        switch (currentState) {
            case INTAKE:
                intake.setPower(-1.0);
                break;
            case OUTAKE:
                intake.setPower(1.0);
                break;
            case IDLE:
            default:
                intake.setPower(0.0);
                break;
        }
    }
}
