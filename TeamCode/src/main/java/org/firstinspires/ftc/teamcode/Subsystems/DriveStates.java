package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class DriveStates extends Subsystem {
    private DRIVE_STATE wantedSuperState = DRIVE_STATE.IDLE;
    private DRIVE_STATE currentSuperState = DRIVE_STATE.IDLE;
    private Drive drive;
    private PID xPID = new PID(1, 0, 0);
    private PID thetaPID = new PID(0.01, 0, 0.001);
    private double DISTANCE_TOLERANCE = 0.1;
    private double THETA_TOLERANCE = 2.0;
    private double distance;
    private double targetTheta;
    private double forwardTargetTheta = 0;

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
        DRIVE_TURN_RIGHT,
        DRIVE_TURN_LEFT
    }

    private DRIVE_STATE handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT: currentSuperState = DRIVE_STATE.DEFAULT; break;
            case IDLE: currentSuperState = DRIVE_STATE.IDLE; break;
            case DRIVE_FORWARD: currentSuperState = DRIVE_STATE.DRIVE_FORWARD; break;
            case DRIVE_TURN_RIGHT: currentSuperState = DRIVE_STATE.DRIVE_TURN_RIGHT; break;
            case DRIVE_TURN_LEFT: currentSuperState = DRIVE_STATE.DRIVE_TURN_LEFT; break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        drive.stop();
    }

    private void handleIdleState() {}

    public void driveForwardDriveDistanceX(double distanceMeters) {
        Mouse.configureOtos();      // reset odometry X to 0 before starting a new forward move
        this.distance = distanceMeters;  // set target distance relative to current position
        forwardTargetTheta = Mouse.getTheta();
    }


    private double driveForwardDistance() {
        return distance; // instead of distance + Mouse.getX()/100
    }

    private void handleDriveForwardState() {
        // Forward control
        xPID.setSetPoint(distance);
        xPID.updatePID(Mouse.getX());
        double forward = xPID.getResult();

        // Heading hold control
        thetaPID.setSetPoint(forwardTargetTheta);
        thetaPID.updatePID(Mouse.getTheta());
        double turn = thetaPID.getResult();

        // Mecanum drive correction:
        // forward + turn on left, forward - turn on right
        drive.drive(
                forward + turn,
                forward - turn,
                -(forward + turn),
                -(forward - turn)
        );
    }


    public void driveTurnDriveDistanceTheta(double degrees) {
        targetTheta = normalizeAngle(Mouse.getTheta() + degrees);
    }

    private void handleDriveTurnRightState() {
        handleDriveTurn(true);  // true = turning right
    }

    private void handleDriveTurnLeftState() {
        handleDriveTurn(false); // false = turning left
    }

    // Unified turning logic
    private void handleDriveTurn (boolean turnRight) {
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
    if (turnRight) {
        drive.drive(-power, power, -power, power);
    } else {
        drive.drive(power, -power, power, -power);
    }
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
            case DRIVE_TURN_RIGHT: handleDriveTurnRightState(); break;
            case DRIVE_TURN_LEFT: handleDriveTurnLeftState(); break;
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