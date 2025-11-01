package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class DriveStates extends Subsystem {
    private DRIVE_STATE wantedSuperState = DRIVE_STATE.IDLE;
    private DRIVE_STATE currentSuperState = DRIVE_STATE.IDLE;
    private Drive drive;
    private PID xPID = new PID(1, 0, 0);
    private PID thetaPID = new PID(0.015, 0, 0.001);
    private double DISTANCE_TOLERANCE = 0.2;
    private double THETA_TOLERANCE = 2.0;
    private double distance;
    private double targetTheta;

    public DriveStates(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        this.drive = new Drive("drive", hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();

        thetaPID.setMinOutput(-1);
        thetaPID.setMaxOutput(1);
        xPID.setMinOutput(-1);
        xPID.setMaxOutput(1);
    }

    public void setWantedState(DRIVE_STATE driveState) {
        wantedSuperState = driveState;
    }

    public enum DRIVE_STATE {
        DEFAULT,
        IDLE,
        DRIVE_FORWARD,
        DRIVE_TURN
    }

    private DRIVE_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT: currentSuperState = DRIVE_STATE.DEFAULT; break;
            case IDLE: currentSuperState = DRIVE_STATE.IDLE; break;
            case DRIVE_FORWARD: currentSuperState = DRIVE_STATE.DRIVE_FORWARD; break;
            case DRIVE_TURN: currentSuperState = DRIVE_STATE.DRIVE_TURN; break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        drive.stop();
    }

    private void handleIdleState() {}

    public void driveForwardDriveDistanceX(double distance) {
        this.distance = distance;
    }

    private double driveForwardDistance() {
        return distance + (Mouse.getX() / 100.0);
    }

    private void handleDriveForwardState() {
        xPID.setSetPoint(driveForwardDistance());
        xPID.updatePID(Mouse.getX());
        drive.drive(xPID.getResult(), xPID.getResult(), -xPID.getResult(), -xPID.getResult());
    }

    public void driveTurnDriveDistanceTheta(double degrees) {
        targetTheta = normalizeAngle(Mouse.getTheta() + degrees);
    }

    private void handleDriveTurnState() {
        double currentTheta = Mouse.getTheta();
        double error = angleError(targetTheta, currentTheta);

        if (Math.abs(error) <= THETA_TOLERANCE) {
            drive.stop();
            return;
        }

        thetaPID.setSetPoint(targetTheta);
        thetaPID.updatePID(currentTheta);

        double power = thetaPID.getResult();
        double scale = Math.min(1.0, Math.abs(error) / 45.0);
        power *= scale;

        power = clamp(power, -0.6, 0.6);
        if (Math.abs(power) < 0.1) power = Math.signum(power) * 0.1;

        drive.drive(-power, power, -power, power);
    }

    public boolean isFinishedX() {
        return Math.abs(Mouse.getX() - distance) <= DISTANCE_TOLERANCE;
    }

    public boolean isFinishedTheta() {
        double error = angleError(targetTheta, Mouse.getTheta());
        return Math.abs(error) <= THETA_TOLERANCE;
    }

    @Override
    public void periodic() {
        Mouse.update();
        handleStateTransitions();
        switch (currentSuperState) {
            case DEFAULT: handleDefaultState(); break;
            case IDLE: handleIdleState(); break;
            case DRIVE_FORWARD: handleDriveForwardState(); break;
            case DRIVE_TURN: handleDriveTurnState(); break;
        }
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double angleError(double target, double current) {
        double error = target - current;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
