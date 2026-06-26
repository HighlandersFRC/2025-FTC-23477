package org.firstinspires.ftc.teamcode.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem;

public class shooterSubsystem extends Subsystem {

    private shooter_states wantedState = shooter_states.IDLE;
    private shooter_states currentState = shooter_states.IDLE;

    DcMotor shooter;
    long shooting_duration;
    boolean isFinished = false;

    public shooterSubsystem (String name) {
        super(name);
    }

    public enum shooter_states {
        SHOOTING,
        IDLE
    }

    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.dcMotor.get("ShooterMotor");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setWantedState(shooter_states shooterState) {this.wantedState = shooterState;}

    private void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
                setWantedState(shooter_states.IDLE);
                break;
            case SHOOTING:
                setWantedState(shooter_states.SHOOTING);
                break;
        }

        currentState = wantedState;
    }

    private void handleIdleState() {
        shooter.setPower(0);
    }

    private void handleShootingState(long duration_seconds) {
        shooter.setPower(0.8);
        sleep(duration_seconds * 1000);
        shooter.setPower(0);
        isFinished = true;
    }

    public void setShootingDuration(long duration) {
        this.shooting_duration = duration;
    }

    public boolean isFinishedShooting() {
        return isFinished;
    }

    public void periodic() {
        handleStateTransitions();
        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;
            case SHOOTING:
                handleShootingState(shooting_duration);
                break;

        }
    }
}
