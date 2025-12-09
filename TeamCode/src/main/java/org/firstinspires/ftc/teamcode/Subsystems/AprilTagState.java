package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class AprilTagState extends Subsystem {

    // ============================
    // STATE MACHINE
    // ============================
    public enum TAG_STATE {
        IDLE,
        RANGE_TRACK
    }

    private TAG_STATE wantedState = TAG_STATE.IDLE;
    private TAG_STATE currentState = TAG_STATE.IDLE;

    public void setWantedState(TAG_STATE state) {
        wantedState = state;
    }

    // ============================
    // LIMELIGHT + PID
    // ============================
    private Limelight3A limelight;
    private Drive drive;

    private double lastTx = 0;

    // PID controllers
    private final PID forwardPID = new PID(1.0, 0.0, 0.01);
    private final PID strafePID  = new PID(1.0, 0.0, 0.01);
    private final PID turnPID    = new PID(0.045, 0.0, 0.030);

    // Exposed drive outputs
    public double forwardPower = 0;
    public double strafePower  = 0;
    public double turnPower    = 0;

    // Targets
    private double desiredDistance = 1.0;
    private double desiredX = 0.0;
    private double desiredTx = 0.0;

    // ============================
    // INIT
    // ============================
    public AprilTagState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        drive = new Drive("drive", hardwareMap);

        forwardPID.setSetPoint(desiredDistance);
        strafePID.setSetPoint(desiredX);
        turnPID.setSetPoint(desiredTx);
    }

    private int stabilityCounter = 0;

    public boolean isAtTarget() {
        // acceptable margins
        double distanceTol = 0.05;  // 5cm
        double xTol = 0.03;         // 3cm
        double txTol = 1.5;         // degrees

        boolean atDistance = Math.abs(forwardPID.getError()) < distanceTol;
        boolean atX = Math.abs(strafePID.getError()) < xTol;
        boolean atHeading = Math.abs(turnPID.getError()) < txTol;

        boolean isAligned = atDistance && atX && atHeading;

        // require the robot to be stable for multiple loops
        if (isAligned) {
            stabilityCounter++;
        } else {
            stabilityCounter = 0;
        }

        return stabilityCounter > 10; // roughly 200ms if loop is 20ms
    }


    // ============================
    // STATE TRANSITION HANDLER
    // ============================
    private TAG_STATE handleTransitions() {
        currentState = wantedState;
        return currentState;
    }

    // ============================
    // IDLE
    // ============================
    private void handleIdle() {
        forwardPower = 0;
        strafePower = 0;
        turnPower = 0;

        double frontLeftPower = -forwardPower + strafePower + turnPower;
        double backLeftPower = -forwardPower - strafePower + turnPower;
        double frontRightPower = -forwardPower - strafePower - turnPower;
        double backRightPower = -forwardPower + strafePower - turnPower;

//        double frontLeftPower = (-rotY + rotX + rx);
//        double backLeftPower = (-rotY - rotX + rx);
//        double frontRightPower = (-rotY - rotX - rx);
//        double backRightPower = (-rotY + rotX - rx);

        drive.drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    // ============================
    // RANGE TRACK (PID)
    // ============================
    private void handleRangeTrack() {
        LLResult result = limelight.getLatestResult();
        boolean hasPose = result != null && result.isValid() && result.getBotpose() != null;

        if (!hasPose) {
            forwardPower = strafePower = turnPower = 0;
            return;
        }

        Pose3D pose = result.getBotpose();
        double x = pose.getPosition().x;  // left/right
        double z = pose.getPosition().z;  // forward/back
        double tx = result.getTx();       // horizontal offset

        // Smooth TX
        double smoothTx = 0.3 * lastTx + 0.7 * tx;
        lastTx = smoothTx;

        // PID calculations
        forwardPower = -forwardPID.updatePID(z);
        strafePower  =  strafePID.updatePID(x);
        turnPower    = -turnPID.updatePID(smoothTx);

        // Slow down when close
        if (z < 0.4) {
            forwardPower *= 0.5;
            turnPower *= 0.5;
        }

        // Clamp
        forwardPower = clamp(forwardPower, -0.8, 0.8);
        strafePower  = clamp(strafePower, -0.6, 0.6);
        turnPower    = clamp(turnPower, -0.5, 0.5);

        double frontLeftPower = -forwardPower + strafePower + turnPower;
        double backLeftPower = -forwardPower - strafePower + turnPower;
        double frontRightPower = -forwardPower - strafePower - turnPower;
        double backRightPower = -forwardPower + strafePower - turnPower;

//        double frontLeftPower = (-rotY + rotX + rx);
//        double backLeftPower = (-rotY - rotX + rx);
//        double frontRightPower = (-rotY - rotX - rx);
//        double backRightPower = (-rotY + rotX - rx);

        drive.drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ============================
    // PERIODIC
    // ============================
    @Override
    public void periodic() {
        handleTransitions();

        switch (currentState) {
            case IDLE:
                handleIdle();
                break;

            case RANGE_TRACK:
                handleRangeTrack();
                break;
        }
    }
}
