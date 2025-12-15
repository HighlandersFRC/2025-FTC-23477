package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.velocityPID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ShooterState extends Subsystem {

    private SHOOTER_STATE wantedSuperState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentSuperState = SHOOTER_STATE.IDLE;
    private DcMotor ShooterMotor;

    private double targetRPM = 0;

    private int ticksPerRev = 28;

    private int lastEncoderPos = 0;
    private long lastTime = 0;
    private double currentTicksPerSecond = 0;

    public ShooterState(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPID.setMinOutput(-1);
        velocityPID.setMaxOutput(1);

        lastTime = System.nanoTime();
        lastEncoderPos = ShooterMotor.getCurrentPosition();
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        velocityPID.setSetPoint(rpm);
        System.out.println("SHOOTER: Target RPM set to " + rpm);
    }



    public double getCurrentRPM() {
        int currentPos = ShooterMotor.getCurrentPosition();
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        int deltaTicks = currentPos - lastEncoderPos;

        if (dt > 0) currentTicksPerSecond = deltaTicks / dt;

        lastEncoderPos = currentPos;
        lastTime = currentTime;

        return (currentTicksPerSecond / ticksPerRev) * 60.0;
    }

    private void runVelocityPID() {
        double currentRPM = getCurrentRPM();
        velocityPID.updatePID(currentRPM);
        ShooterMotor.setPower(velocityPID.getResult());
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
//        setTargetRPM(3000);
//        runVelocityPID();

        ShooterMotor.setPower(0.6);
    }

    private void handleIdleState() {
        ShooterMotor.setPower(0);
    }

    private void handleShootingState() {
        runVelocityPID();
        System.out.println("Current RPM: " + getCurrentRPM());
    }

    private void handleJammedState() {
        ShooterMotor.setPower(-0.267);
    }

    public boolean isAtTargetVelocity() {
        double currentRPM = getCurrentRPM();
        return Math.abs(targetRPM - currentRPM) < 100; // within 100 RPM tolerance
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