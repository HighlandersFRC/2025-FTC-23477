package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Tools.Constants.FLYWHEEL_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MOTOR_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOTER_LOOKUP;
import static org.firstinspires.ftc.teamcode.Tools.Constants.STABLE_DURATION_MS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.TARGET_RPM;
import static org.firstinspires.ftc.teamcode.Tools.Constants.VELOCITY_PID;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Limelight;


public class ShooterState extends Subsystem {
    public DcMotorEx shooterMotor;

    private SHOOTER_STATE wantedState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentState = SHOOTER_STATE.IDLE;

    private long stableStartTime = 0;

    private double currentRPM = 0;


    public ShooterState(String name) {
        super(name);
    }


    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "ShooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        VELOCITY_PID.setMinOutput(-1);
        VELOCITY_PID.setMaxOutput(1);


        Limelight.init(hardwareMap);
    }


    public void setWantedState(SHOOTER_STATE state) {
        wantedState = state;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(TARGET_RPM - computeRPM()) < 50;
    }

    public boolean isAtTargetVelocityStable() {
        double error = Math.abs(TARGET_RPM - computeRPM());
        if (error < 50) {
            if (stableStartTime == 0) {
                stableStartTime = System.currentTimeMillis();
            }
            return System.currentTimeMillis() - stableStartTime >= STABLE_DURATION_MS;
        } else {
            stableStartTime = 0;
            return false;
        }
    }




    public double getTargetRPM() {
        return TARGET_RPM;
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
                VELOCITY_PID.reset();
            }
            currentState = wantedState;
        }
    }

    private void idleLoop() {
        shooterMotor.setPower(-0.5);
    }

    @SuppressLint("DefaultLocale")
    private void shootLoop() {
        currentRPM = computeRPM();
       runVelocityPID();
    }



    private void runVelocityPID() {
        VELOCITY_PID.setSetPoint(TARGET_RPM);
        VELOCITY_PID.updatePID(currentRPM);
        shooterMotor.setPower(-VELOCITY_PID.getResult());
    }

//    public double computeRPM() {
//        int currentPos = -shooterMotor.getCurrentPosition();
//        long currentTime = System.nanoTime();
//
//        double dt = (currentTime - lastTime) / 1e9;
//        int deltaTicks = currentPos - lastEncoderPos;
//
//        if (dt < MIN_DT) {
//            return currentRPM; // prevent spike
//        }
//
//        lastEncoderPos = currentPos;
//        lastTime = currentTime;
//
//        double ticksPerSecond = deltaTicks / dt;
//        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
//    }

public double computeRPM() {
        double motorRPS = -shooterMotor.getVelocity() / MOTOR_TICKS_PER_REV;
        return motorRPS * 60.0 / FLYWHEEL_GEAR_RATIO;
}

    @SuppressLint("DefaultLocale")
    public void setTargetRPMFromDistance(double distance) {
        double rpm = getRPMFromDistance(distance);
        TARGET_RPM = rpm;
        VELOCITY_PID.setSetPoint(rpm);
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
