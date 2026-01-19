package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOTER_LOOKUP;
import static org.firstinspires.ftc.teamcode.Tools.Constants.targetRPM;
import static org.firstinspires.ftc.teamcode.Tools.Constants.velocityPID;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.Limelight;


public class ShooterState extends Subsystem {
    public DcMotor shooterMotor;

    private SHOOTER_STATE wantedState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentState = SHOOTER_STATE.IDLE;

    private int lastEncoderPos = 0;
    private long lastTime = 0;

    // FIX: cache RPM once per loop
    private double currentRPM = 0;

    private static final int TICKS_PER_REV = 28;
    private static final double MIN_DT = 0.02; // 20

    public ShooterState(String name) {
        super(name);
    }


    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.dcMotor.get("ShooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        velocityPID.setMinOutput(-1);
        velocityPID.setMaxOutput(1);

        lastTime = System.nanoTime();
        lastEncoderPos = shooterMotor.getCurrentPosition();
        Limelight.init(hardwareMap);
    }


    public void setWantedState(SHOOTER_STATE state) {
        wantedState = state;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(targetRPM - computeRPM()) < 100;
    }

    public double getTargetRPM() {
        return targetRPM;
    }


    @Override
    public void periodic() {
        handleStateTransition();

        switch (currentState) {
            case IDLE:
                idleLoop();
                break;

            case SHOOT:
                shootLoop();
                break;

            case DEFAULT:
                shooterMotor.setPower(0.6);
                break;
        }
    }

    private void handleStateTransition() {
        if (currentState != wantedState) {
            if (wantedState == SHOOTER_STATE.SHOOT) {
                velocityPID.reset();
            }
            currentState = wantedState;
        }
    }

    private void idleLoop() {
        shooterMotor.setPower(-0.25);
    }

    @SuppressLint("DefaultLocale")
    private void shootLoop() {
        currentRPM = computeRPM();
       runVelocityPID();
    }



    private void runVelocityPID() {
        velocityPID.setSetPoint(targetRPM);
        velocityPID.updatePID(currentRPM);
        shooterMotor.setPower(-velocityPID.getResult());
    }

    public double computeRPM() {
        int currentPos = -shooterMotor.getCurrentPosition();
        long currentTime = System.nanoTime();

        double dt = (currentTime - lastTime) / 1e9;
        int deltaTicks = currentPos - lastEncoderPos;

        if (dt < MIN_DT) {
            return currentRPM; // prevent spike
        }

        lastEncoderPos = currentPos;
        lastTime = currentTime;

        double ticksPerSecond = deltaTicks / dt;
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }



    @SuppressLint("DefaultLocale")
    public void setTargetRPMFromDistance(double distance) {
        double rpm = getRPMFromDistance(distance);
        targetRPM = rpm;
        velocityPID.setSetPoint(rpm);
    }

    public double interpolateRPM(double d1, double r1, double d2, double r2, double distance) {
        double RPM;

        RPM = (r2 - r1) * (distance - d1) / (d2 - d1) + r1;

        return RPM;
    }

    public double getRPMFromDistance(double distance) {

        if (distance <= SHOOTER_LOOKUP[0][0]) {
            return SHOOTER_LOOKUP[0][1];
        }
        // -2 because if we didn't we would looking into a unknown value we need 1 so we do i+1 to end
        for (int i = 0; i <= SHOOTER_LOOKUP.length - 2; i++) {
            if (SHOOTER_LOOKUP[i][0] <= distance && SHOOTER_LOOKUP[i + 1][0] > distance) {
                return interpolateRPM(SHOOTER_LOOKUP[i][0],SHOOTER_LOOKUP[i][1],SHOOTER_LOOKUP[i+1][0],SHOOTER_LOOKUP[i+1][1],distance);
            }
        }

        return SHOOTER_LOOKUP[SHOOTER_LOOKUP.length - 1][1];
    }

    public enum SHOOTER_STATE {
        IDLE,
        SHOOT,
        DEFAULT
    }
}
