package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem;

public class shooterSubsystem extends Subsystem {
    private shooter_states wantedState = shooter_states.IDLE;
    private shooter_states currentState = shooter_states.IDLE;

    private DcMotor shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private double shootingDurationSeconds;

    public shooterSubsystem(String name) {
        super(name);
    }

    public enum shooter_states {
        SHOOTING,
        TELEOP,
        IDLE
    }

    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.dcMotor.get("ShooterMotor");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setWantedState(shooter_states shooterState) {
        if (shooterState != wantedState) {
            timer.reset();
        }

        wantedState = shooterState;
    }

    public void setShootingDuration(double durationSeconds) {
        shootingDurationSeconds = durationSeconds;
    }

    public boolean isFinishedShooting() {
        return currentState == shooter_states.SHOOTING
                && timer.seconds() >= shootingDurationSeconds;
    }

    public void periodic() {
        currentState = wantedState;

        switch (currentState) {
            case SHOOTING:
            case TELEOP:
                shooter.setPower(-0.8);
                break;
            case IDLE:
            default:
                shooter.setPower(0.0);
                break;
        }
    }
}
