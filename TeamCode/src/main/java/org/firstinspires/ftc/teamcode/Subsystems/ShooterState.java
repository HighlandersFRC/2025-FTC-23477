package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.PIDF;

import java.util.Arrays;

public class ShooterState extends Subsystem {

    private SHOOTER_STATE wantedSuperState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentSuperState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE previousSuperState = SHOOTER_STATE.IDLE;
    private DcMotor ShooterMotor;

    private PIDF velocityPID;
    private double targetRPM = 0;

    private int ticksPerRev = 28;

    private int lastEncoderPos = 0;
    private long lastTime = 0;

    // Simplified velocity measurement - less filtering
    private double filteredRPM = 0;
    private final double ALPHA = 0.5; // Less aggressive filtering

    private static final int VELOCITY_SAMPLES = 3; // Fewer samples for faster response
    private double[] velocitySamples = new double[VELOCITY_SAMPLES];
    private int sampleIndex = 0;

    private Limelight3A limelight;
    private boolean useAprilTagAdjustment = false;
    private boolean aprilTagDetected = false;

    private double feedForward = (double) 1 / 6000;

    public ShooterState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // PURE FEEDFORWARD + TINY P correction only
        // No I or D to prevent oscillation
        velocityPID = new PIDF(0.005, 0, 0, feedForward);
        velocityPID.setMinOutput(-1);
        velocityPID.setMaxOutput(1);

        lastTime = System.nanoTime();
        lastEncoderPos = ShooterMotor.getCurrentPosition();
        filteredRPM = 0;

        Arrays.fill(velocitySamples, 0);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        velocityPID.setSetPoint(rpm);
        System.out.println("SHOOTER: Target RPM set to " + rpm);
    }

    public boolean isAprilTagDetected() {
        return aprilTagDetected;
    }

    public double getCurrentRPM() {
        int currentPos = ShooterMotor.getCurrentPosition();
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        int deltaTicks = currentPos - lastEncoderPos;

        if (dt > 0.015) { // 15ms threshold
            double ticksPerSecond = deltaTicks / dt;
            lastEncoderPos = currentPos;
            lastTime = currentTime;

            double rawRPM = (ticksPerSecond / ticksPerRev) * 60.0;

            velocitySamples[sampleIndex] = rawRPM;
            sampleIndex = (sampleIndex + 1) % VELOCITY_SAMPLES;

            double avgRPM = 0;
            for (double sample : velocitySamples) {
                avgRPM += sample;
            }
            avgRPM /= VELOCITY_SAMPLES;

            filteredRPM = ALPHA * avgRPM + (1 - ALPHA) * filteredRPM;
        }

        return filteredRPM;
    }

    private void runVelocityPID() {
        double currentRPM = getCurrentRPM();
        velocityPID.updatePID(currentRPM);
        ShooterMotor.setPower(velocityPID.getResult());
        System.out.println("Current RPM : " + currentRPM + "RPM");
    }

    private SHOOTER_STATE handleStateTransitions() {
        if (wantedSuperState != previousSuperState) {
            velocityPID.reset();
            previousSuperState = wantedSuperState;
        }

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
        velocityPID.setSetPoint(2900);
        runVelocityPID();

//        ShooterMotor.setPower(0);
    }

    private void handleIdleState() {
        ShooterMotor.setPower(0);
        filteredRPM = 0;
        velocityPID.reset();
        for (int i = 0; i < VELOCITY_SAMPLES; i++) {
            velocitySamples[i] = 0;
        }
        sampleIndex = 0;
    }

    private void handleShootingState() {
        runVelocityPID();
    }

    private void handleJammedState() {
        velocityPID.setSetPoint(-500);
        runVelocityPID();
    }

    public boolean isAtTargetVelocity() {
        double currentRPM = getCurrentRPM();
        return Math.abs(targetRPM - currentRPM) < 200;
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