package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem;

public class indexerSubsystem extends Subsystem {
    private states wantedState = states.IDLE;
    private states currentState = states.IDLE;

    private DcMotor indexer;
    private DcMotor queuer;
    private final ElapsedTime timer = new ElapsedTime();
    private double indexDurationSeconds;

    public indexerSubsystem(String name) {
        super(name);
    }

    public enum states {
        IDLE,
        INDEXING
    }

    public void init(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotor.class, "IndexerMotor");
        queuer = hardwareMap.get(DcMotor.class, "QueueMotor");

        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        queuer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        queuer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setWantedState(states state) {
        if (state != wantedState) {
            timer.reset();
        }

        wantedState = state;
    }

    public void setIndexDuration(double durationSeconds) {
        indexDurationSeconds = durationSeconds;
    }

    public boolean isFinished() {
        return currentState == states.INDEXING
                && timer.seconds() >= indexDurationSeconds;
    }

    public void periodic() {
        currentState = wantedState;

        switch (currentState) {
            case INDEXING:
                indexer.setPower(1.0);
                queuer.setPower(1.0);
                break;
            case IDLE:
            default:
                indexer.setPower(0.0);
                queuer.setPower(0.0);
                break;
        }
    }
}
