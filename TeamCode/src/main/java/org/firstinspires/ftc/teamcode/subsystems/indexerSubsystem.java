package org.firstinspires.ftc.teamcode.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem;

public class indexerSubsystem extends Subsystem {

    private states wantedState = states.IDLE;
    private states currentState = states.IDLE;

    DcMotor indexer;
    DcMotor queuer;
    long index_duration;
    long time_passed;

    public indexerSubsystem(String name) {super(name);}

    public void init(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotor.class, "IndexerMotor");
        queuer = hardwareMap.get(DcMotor.class, "QueueMotor");
    }

    public enum states {
        IDLE,
        INDEXING
    }

    public void setWantedState(states state) {
        this.wantedState = state;
    }

    private void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
                setWantedState(states.IDLE);
                break;
            case INDEXING:
                setWantedState(states.INDEXING);
                break;
        }

        currentState = wantedState;
    }

    private void handleIdleState() {
        indexer.setPower(0);
    }

    private void handleIndexState() {
        indexer.setPower(1);
        queuer.setPower(1);
    }

    public void setIndexDuration(long duration) {
        this.index_duration = duration;
    }

    public boolean isFinished() {
       for (int time = 0; time <= index_duration; time++) {
           time_passed = time;
           sleep(1000);
       }

       return time_passed == index_duration;
    }

    public void periodic() {
        handleStateTransitions();
        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;
            case INDEXING:
                handleIndexState();
                break;
        }
    }


}
