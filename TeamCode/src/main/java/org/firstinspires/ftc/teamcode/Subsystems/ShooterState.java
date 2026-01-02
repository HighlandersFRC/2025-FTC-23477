package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.velocityPID;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tools.Limelight;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.stream.Stream;

public class ShooterState extends Subsystem {


    private DcMotor shooterMotor;


    private SHOOTER_STATE wantedState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE currentState = SHOOTER_STATE.IDLE;
    private SHOOTER_STATE lastState = null;


    private double targetRPM = 0;

    private int lastEncoderPos = 0;
    private long lastTime = 0;
    private double ticksPerSecond = 0;


    private static final double[][] SHOOTER_LOOKUP = {
            {0.0, 3000},
            {1.341, 3500.0},
            {1.6378, 3917.9567},
            {2.571, 6000}
    };


    public ShooterState(String name) {
        super(name);
    }


    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.dcMotor.get("ShooterMotor");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPID.setMinOutput(-1);
        velocityPID.setMaxOutput(1);

        lastTime = System.nanoTime();
        lastEncoderPos = shooterMotor.getCurrentPosition();
        Limelight.init(hardwareMap);

//        [0.0, 3000.0],
//    [1.341, 3500.0],
//    [1.6378, 3917.9567],
//    [2.571, 6000]

    }


    public void setWantedState(SHOOTER_STATE state) {
        wantedState = state;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(targetRPM - getCurrentRPM()) < 100;
    }

    public double getTargetRPM() {
        return targetRPM;
    }


    @Override
    public void periodic() {
        handleStateTransition();

        if (currentState != lastState) {
            onStateEnter(currentState);
            lastState = currentState;
        }

        switch (currentState) {
            case IDLE:
                idleLoop();
                break;

            case SHOOT:
                shootLoop();
                break;

            case JAMMED:
                jammedLoop();
                break;

            case DEFAULT:
                defaultLoop();
                break;
        }
    }

    private void handleStateTransition() {
        currentState = wantedState;
    }


    private void onStateEnter(SHOOTER_STATE state) {
        switch (state) {
            case IDLE:
                shooterMotor.setPower(0);
                targetRPM = 0;
                break;

            case SHOOT:
                velocityPID.reset();
                System.out.println("SHOOTER: Entering SHOOT state");
                break;

            case JAMMED:
                shooterMotor.setPower(-0.27);
                break;

            case DEFAULT:
                shooterMotor.setPower(0.6);
                break;
        }
    }

    private void idleLoop() {
        targetRPM = 1000;
        runVelocityPID();
    }

    @SuppressLint("DefaultLocale")
    private void shootLoop() {
        runVelocityPID();
    }

    private void jammedLoop() {
        shooterMotor.setPower(-0.27);
    }

    private void defaultLoop() {
        shooterMotor.setPower(0.6);
    }


    private void runVelocityPID() {
        double currentRPM = getCurrentRPM();
        velocityPID.setSetPoint(targetRPM);
        velocityPID.updatePID(currentRPM);
        shooterMotor.setPower(velocityPID.getResult());
    }

    public double getCurrentRPM() {
        int currentPos = shooterMotor.getCurrentPosition();
        long currentTime = System.nanoTime();

        double dt = (currentTime - lastTime) / 1e9;
        int deltaTicks = currentPos - lastEncoderPos;

        if (dt > 0) {
            ticksPerSecond = deltaTicks / dt;
        }

        lastEncoderPos = currentPos;
        lastTime = currentTime;

        int ticksPerRev = 28;
        return (ticksPerSecond / ticksPerRev) * 60.0;
    }


    @SuppressLint("DefaultLocale")
    public void setTargetRPMFromDistance(double distance) {
        double rpm = getRPMFromDistance(distance);
        targetRPM = rpm;
        targetRPM = targetRPM * 1.19;
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
        JAMMED,
        DEFAULT
    }
}
