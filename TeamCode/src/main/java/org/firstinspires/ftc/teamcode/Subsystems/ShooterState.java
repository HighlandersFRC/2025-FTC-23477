package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.PID;

import java.util.List;

public class ShooterState extends Subsystem {

    private SHOOTER_STATE wantedSuperState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentSuperState = SHOOTER_STATE.IDLE;
    private DcMotor ShooterMotor;

    private PID velocityPID;
    private double targetRPM = 0;
    private double targetTicks = 0;
    private boolean reachedTarget = false;

    private int ticksPerRev = 28;

    private int lastEncoderPos = 0;
    private long lastTime = 0;
    private double currentTicksPerSecond = 0;

    // Limelight integration
    private Limelight3A limelight;
    private boolean useAprilTagAdjustment = false;
    private boolean aprilTagDetected = false;

    // Camera configuration for distance calculation
    private double cameraHeightInches = 12.0;  // ADJUST: Your camera height from ground
    private double targetHeightInches = 18.0;  // ADJUST: AprilTag height from ground
    private double cameraAngleDegrees = 15.0;  // ADJUST: Camera upward tilt angle

    public ShooterState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPID = new PID(0.0005, 0.00001, 0.0001);
        velocityPID.setMinOutput(-1);
        velocityPID.setMaxOutput(1);

        lastTime = System.nanoTime();
        lastEncoderPos = ShooterMotor.getCurrentPosition();

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Switch to AprilTag pipeline
            limelight.start();
            System.out.println("SHOOTER: Limelight initialized successfully");
        } catch (Exception e) {
            // Limelight not found, continue without it
            limelight = null;
            System.out.println("SHOOTER: Limelight not found - " + e.getMessage());
        }
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        velocityPID.setSetPoint(rpm);
        System.out.println("SHOOTER: Target RPM set to " + rpm);
    }

    public void setTargetTicks(double ticks) {
        targetTicks = ticks;
        reachedTarget = false;
        ShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastEncoderPos = 0;
        lastTime = System.nanoTime();
        System.out.println("SHOOTER: Target ticks set to " + ticks);
    }

    public void enableAprilTagAdjustment(boolean enable) {
        useAprilTagAdjustment = enable;
        System.out.println("SHOOTER: AprilTag adjustment " + (enable ? "ENABLED" : "DISABLED"));
    }

    public void setCameraConfig(double cameraHeight, double targetHeight, double cameraAngle) {
        this.cameraHeightInches = cameraHeight;
        this.targetHeightInches = targetHeight;
        this.cameraAngleDegrees = cameraAngle;
        System.out.println("SHOOTER: Camera config set - Height: " + cameraHeight +
                ", Target Height: " + targetHeight + ", Angle: " + cameraAngle);
    }

    public boolean hasReachedTargetTicks() {
        return reachedTarget;
    }

    public boolean isAprilTagDetected() {
        return aprilTagDetected;
    }

    private double getCurrentRPM() {
        int currentPos = ShooterMotor.getCurrentPosition();
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        int deltaTicks = currentPos - lastEncoderPos;

        if (dt > 0) currentTicksPerSecond = deltaTicks / dt;

        lastEncoderPos = currentPos;
        lastTime = currentTime;

        return (currentTicksPerSecond / ticksPerRev) * 60.0;
    }

    private void updateTargetFromAprilTag() {
        aprilTagDetected = false;

        if (limelight == null) {
            System.out.println("SHOOTER: Limelight is null");
            return;
        }

        if (!useAprilTagAdjustment) {
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            System.out.println("SHOOTER: No valid Limelight result");
            return;
        }

        // Get AprilTag results
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            System.out.println("SHOOTER: No AprilTags detected");
            return;
        }

        // Use the closest/most visible tag
        LLResultTypes.FiducialResult bestTag = fiducials.get(0);
        int tagId = bestTag.getFiducialId();
        aprilTagDetected = true;
        System.out.println("SHOOTER: Detected AprilTag ID: " + tagId);

        // Get horizontal and vertical angles to target
        double targetXDegrees = bestTag.getTargetXDegrees();
        double targetYDegrees = bestTag.getTargetYDegrees();

        // Calculate distance based on vertical angle
        double angleToTarget = cameraAngleDegrees + targetYDegrees;
        double heightDiff = targetHeightInches - cameraHeightInches;
        double distance = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTarget)));

        // Calculate RPM based on distance using the formula
        double calculatedRPM = calculateRPMFromDistance(distance);
        System.out.println("SHOOTER: Tag " + tagId + " at " + distance + " inches -> " + calculatedRPM + " RPM");
        setTargetRPM(calculatedRPM);
    }

    private double calculateRPMFromDistance(double distance) {

        double a = 1;
        double b = 30.0;
        double c = 2000.0;

        double calculatedRPM = a * distance * distance + b * distance + c;

        double minRPM = 0000.0;
        double maxRPM = 6000.0;

        double clampedRPM = Math.max(minRPM, Math.min(maxRPM, calculatedRPM));

        System.out.println("SHOOTER: Raw calculated RPM: " + calculatedRPM + ", Clamped: " + clampedRPM);

        return clampedRPM;
    }

    private void runVelocityPID() {
        double currentRPM = getCurrentRPM();
        double output = velocityPID.updatePID(currentRPM);
        ShooterMotor.setPower(output);
    }

    private SHOOTER_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = SHOOTER_STATE.DEFAULT;
                break;
            case IDLE:
                currentSuperState = SHOOTER_STATE.IDLE;
                break;
            case SHOOT:
                currentSuperState = SHOOTER_STATE.SHOOT;
                break;
            case JAMMED:
                currentSuperState = SHOOTER_STATE.JAMMED;
                break;
        }
        return currentSuperState;
    }

    public void setWantedState(SHOOTER_STATE shooterState) {
        wantedSuperState = shooterState;
    }

    private void handleDefaultState() {
        ShooterMotor.setPower(0);
    }

    private void handleIdleState() {
        ShooterMotor.setPower(0);
    }

    private void handleShootingState() {
        // Update target RPM based on AprilTag if enabled (only adjusts RPM, doesn't stop shooter)
        updateTargetFromAprilTag();

        // Always run velocity PID - just like the original code
        runVelocityPID();

        if (Math.abs(ShooterMotor.getCurrentPosition()) >= targetTicks) {
            System.out.println("SHOOTER: Reached target ticks, stopping");
            ShooterMotor.setPower(0);
            reachedTarget = true;
        }
    }

    private void handleJammedState() {
        ShooterMotor.setPower(-0.267);
    }

    public boolean isAtTargetVelocity() {
        double currentRPM = getCurrentRPM();
        return Math.abs(targetRPM - currentRPM) < 100; // within 100 RPM tolerance
    }

    public LLResult getLimelightResult() {
        return limelight != null ? limelight.getLatestResult() : null;
    }

    public double getCurrentTargetRPM() {
        return targetRPM;
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
            case SHOOT:
                handleShootingState();
                break;
            case JAMMED:
                handleJammedState();
                break;
        }
    }

    public enum SHOOTER_STATE {
        DEFAULT,
        IDLE,
        SHOOT,
        JAMMED
    }
}