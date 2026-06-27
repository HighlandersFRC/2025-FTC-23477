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
    double time_passed;

    public shooterSubsystem (String name) {
        super(name);
    }

    public enum shooter_states {
        SHOOTING,
        TELEOP,
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
            case TELEOP:
                setWantedState(shooter_states.TELEOP);
        }

        currentState = wantedState;
    }

    private void handleIdleState() {
        shooter.setPower(0);
    }

    private void handleShootingState() {
        shooter.setPower(0.8);
    }

    public void setShootingDuration(long duration) {
        this.shooting_duration = duration;
    }

    public boolean isFinishedShooting() {
        for (int time = 0; time <= shooting_duration; time++ ) {
            sleep(1000);
            time_passed = time;
        }
        return time_passed == shooting_duration;
    }

    public void periodic() {
        handleStateTransitions();
        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;
            case SHOOTING:
                handleShootingState();
                break;

        }
    }
}
