package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class AprilTagState extends Subsystem {



    public enum APRIL_TAG_STATE {
        IDLE,
        RANGE_TRACK
    }

    private APRIL_TAG_STATE wantedState = APRIL_TAG_STATE.IDLE;
    private APRIL_TAG_STATE currentState = APRIL_TAG_STATE.IDLE;

    public void setWantedState(APRIL_TAG_STATE state) {
        wantedState = state;
    }

    private Limelight3A limelight;
    private Drive drive;

    // smoothing / caching
    private double lastTx = 0.0;
    private double[] lastGoodPose = null; // [x, y, z]
    private int invalidFrameCount = 0;
    private final int maxInvalidFrames = 5;

    // PIDs (same as original instantiation)
    private final PID forwardPID = new PID(1.0, 0.0, 0.01);
    private final PID strafePID = new PID(1.0, 0.0, 0.01);
    private final PID turnPID = new PID(0.045, 0.0, 0.030);

    // outputs
    public double forwardPower = 0;
    public double strafePower = 0;
    public double turnPower = 0;

    // targets
    private double desiredDistance = 1.0;
    private double desiredX = 0.0;
    private double desiredTx = 0.0;

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
        double distanceTol = 0.05;
        double xTol = 0.03;
        double txTol = 1.5;

        boolean atDistance = Math.abs(forwardPID.getError()) < distanceTol;
        boolean atX = Math.abs(strafePID.getError()) < xTol;
        boolean atHeading = Math.abs(turnPID.getError()) < txTol;

        boolean isAligned = atDistance && atX && atHeading;

        if (isAligned) {
            stabilityCounter++;
        } else {
            stabilityCounter = 0;
        }

        return stabilityCounter > 10;
    }

    private APRIL_TAG_STATE handleTransitions() {
        currentState = wantedState;
        return currentState;
    }

    private void handleIdle() {
        forwardPower = 0;
        strafePower = 0;
        turnPower = 0;

        double frontLeftPower = -forwardPower + strafePower + turnPower;
        double backLeftPower = -forwardPower - strafePower + turnPower;
        double frontRightPower = -forwardPower - strafePower - turnPower;
        double backRightPower = -forwardPower + strafePower - turnPower;

        drive.drive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void handleRangeTrack() {
        LLResult result = limelight.getLatestResult();
        boolean hasValidResult = result != null && result.isValid();

        double x = Double.NaN;
        double z = Double.NaN;
        double tx = Double.NaN;

        if (hasValidResult) {
            Pose3D pose = result.getBotpose();
            if (pose != null) {
                x = pose.getPosition().x;
                z = pose.getPosition().z;
                lastGoodPose = new double[]{x, pose.getPosition().y, z};
            }

            /* getTx() should exist per your original code usage*/
            tx = result.getTx();
        }

        // use last good pose for short dropouts
    }
}