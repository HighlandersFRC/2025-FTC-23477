package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DISTANCE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MAX_TURN;
import static org.firstinspires.ftc.teamcode.Tools.Constants.MIN_TURN;
import static org.firstinspires.ftc.teamcode.Tools.Constants.THETA_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Tools.Constants.TX_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Tools.Constants.LAST_TX;
import static org.firstinspires.ftc.teamcode.Tools.Constants.THETA_PID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.THETA_PID_LIMELIGHT;
import static org.firstinspires.ftc.teamcode.Tools.Constants.X_PID;
import static org.firstinspires.ftc.teamcode.Tools.Constants.Y_PID;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

public class DriveStates extends Subsystem {
    private DRIVE_STATE wantedSuperState = DRIVE_STATE.IDLE;
    private DRIVE_STATE currentSuperState = DRIVE_STATE.IDLE;
    private Drive drive;

    public IMU imu;


    private double distanceX;
    private double distanceY;
    private double targetTheta;


    public DriveStates(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        this.drive = new Drive("drive", hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();


        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);




        Limelight.init(hardwareMap);

        THETA_PID.setMinOutput(-1);
        THETA_PID.setMaxOutput(1);

        X_PID.setMinOutput(-1);
        X_PID.setMaxOutput(1);

        Y_PID.setMinOutput(-1);
        Y_PID.setMaxOutput(1);

        THETA_PID_LIMELIGHT.setSetPoint(0.0);
    }

    public void setWantedState(DRIVE_STATE driveState) {
        wantedSuperState = driveState;
    }

    public enum DRIVE_STATE {
        DEFAULT,
        IDLE,
        DRIVE_FORWARD,
        DRIVE_TURN_RIGHT,
        DRIVE_TURN_LEFT,
        DRIVE_STRAFE,
        AUTO_TURN,
        DRIVE_FORWARD_TIME,
        DRIVE_TURN_LEFT_TIME,
        DRIVE_TURN_RIGHT_TIME,
        DRIVE_STRAFE_TIME,
        DRIVE_BACK_TIME,
        DRIVE_STRAFE_BACK_TIME
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT: currentSuperState = DRIVE_STATE.DEFAULT; break;
            case IDLE: currentSuperState = DRIVE_STATE.IDLE; break;
            case DRIVE_FORWARD: currentSuperState = DRIVE_STATE.DRIVE_FORWARD; break;
            case DRIVE_TURN_RIGHT: currentSuperState = DRIVE_STATE.DRIVE_TURN_RIGHT; break;
            case DRIVE_TURN_LEFT: currentSuperState = DRIVE_STATE.DRIVE_TURN_LEFT; break;
            case DRIVE_STRAFE: currentSuperState = DRIVE_STATE.DRIVE_STRAFE; break;
            case AUTO_TURN: currentSuperState = DRIVE_STATE.AUTO_TURN; break;
            case DRIVE_FORWARD_TIME: currentSuperState = DRIVE_STATE.DRIVE_FORWARD_TIME; break;
            case DRIVE_STRAFE_TIME: currentSuperState = DRIVE_STATE.DRIVE_STRAFE_TIME; break;
            case DRIVE_TURN_LEFT_TIME: currentSuperState = DRIVE_STATE.DRIVE_TURN_LEFT_TIME; break;
            case DRIVE_TURN_RIGHT_TIME: currentSuperState = DRIVE_STATE.DRIVE_TURN_RIGHT_TIME; break;
            case DRIVE_BACK_TIME: currentSuperState = DRIVE_STATE.DRIVE_BACK_TIME; break;
            case DRIVE_STRAFE_BACK_TIME: currentSuperState = DRIVE_STATE.DRIVE_STRAFE_BACK_TIME; break;
        }
    }

    private void handleDefaultState() {
        drive.stop();
    }



    private void handleIdleState() {}
// DRIVE WITH MOUSE SENSOR
    public void driveForwardDriveDistanceX(double distanceMeters) {
        Mouse.configureOtos();
        this.distanceX = distanceMeters;
    }

    public void driveForwardDriveDistanceY(double distanceMeters) {
        Mouse.configureOtos();
        this.distanceY = distanceMeters;
    }

    private double driveForwardDistance() {
        return distanceX;
    }

    private double driveStrafeDistance() {
        return distanceY;
    }

    private void handleDriveForwardState() {
        X_PID.setSetPoint(driveForwardDistance());
        X_PID.updatePID(Mouse.getX());

        drive.drive(-X_PID.getResult(), X_PID.getResult(), X_PID.getResult(), -X_PID.getResult());
    }

    private void handleStrafeState() {
        Y_PID.setSetPoint(driveStrafeDistance());
        Y_PID.updatePID(Mouse.getY());

        drive.drive(-Y_PID.getResult(), -Y_PID.getResult(), -Y_PID.getResult(), -Y_PID.getResult());
    }

    public void driveTurnDriveDistanceTheta(double degrees) {
        targetTheta = normalizeAngle(Mouse.getTheta() + degrees);
    }



    private void handleDriveTurn (boolean turnRight) {
        double currentTheta = Mouse.getTheta();
        double error = angleError(targetTheta, currentTheta);

        if (Math.abs(error) <= THETA_TOLERANCE) {
            drive.stop();
            return;
        }

        THETA_PID.setSetPoint(targetTheta);
        THETA_PID.updatePID(currentTheta);

        double power = THETA_PID.getResult();
        double scale = Math.min(1.0, Math.abs(error) / 45.0);
        power *= scale;

        power = clamp(power);
        if (Math.abs(power) < 0.1) power = Math.signum(power) * 0.1;
        if (turnRight) {
            drive.drive(-power, -power, power, -power);
        } else {
            drive.drive(power, power, -power, power);
        }
    }

    private void handleAutoTurnState() {
        LLResult result = Limelight.getResult();
        if (result != null && result.isValid()) {

            double tx = Limelight.getTx();


            double smoothTx = 0.3 * LAST_TX + 0.7 * tx;
            LAST_TX = smoothTx;

            double turnPower = -THETA_PID_LIMELIGHT.updatePID(smoothTx);

            if (turnPower != 0 && Math.abs(turnPower) < MIN_TURN) {
                turnPower = Math.signum(turnPower) * MIN_TURN;
            }

            turnPower = Math.max(-MAX_TURN, Math.min(MAX_TURN, turnPower));

            drive.drive(turnPower, -turnPower, -turnPower, -turnPower);

        }
    }

    public boolean isFinishedAutoTurnTheta() {
        double tx = Limelight.getTx();
        double smoothTx = 0.3 * LAST_TX + 0.7 * tx;
        LAST_TX = smoothTx;
        return Math.abs(smoothTx) < TX_TOLERANCE;
    }

    public boolean isFinishedX() {
        return Math.abs(Mouse.getX() - distanceX) <= DISTANCE_TOLERANCE;
    }

    public boolean isFinishedY(){
        return Math.abs(Mouse.getY() - distanceY) <= DISTANCE_TOLERANCE;
    }

    public boolean isFinishedTheta() {
        double error = angleError(targetTheta, Mouse.getTheta());
        return Math.abs(error) <= THETA_TOLERANCE;
    }

    // DRIVE WITH TIME

    private void handleDriveTimeState() {
        drive.drive(0.5,0.5,-0.5,0.5);
    }
    private void handleDriveBackTimeState() {
        drive.drive(-0.5,-0.5,0.5,-0.5);
    }
    private void handleStrafeTimeState() {
        drive.drive(0.5,-0.5,0.5,0.5);
    }
    private void handleStrafeBackTimeState() {
        drive.drive(-0.5,0.5,-0.5,-0.5);
    }



    private double getHeading() {
        return normalizeAngle(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    public void driveTurnDriveDistanceThetaIMU(double degrees) {
        targetTheta = getHeading() + degrees;
    }
    private void handleDriveTurnIMU() {
        double currentTheta = getHeading();
        double error = angleError(targetTheta, currentTheta);

        // Stop if we're close enough
        if (Math.abs(error) <= THETA_TOLERANCE) {
            drive.stop();
            return;
        }

        THETA_PID.setSetPoint(targetTheta);
        THETA_PID.updatePID(currentTheta);

        double power = THETA_PID.getResult();

        // Scale power as we get close (prevents overshoot)
        double scale = Math.min(1.0, Math.abs(error) / 45.0);
        power *= scale;

        // Clamp + minimum power
        power = clamp(power);
        if (Math.abs(power) < 0.1) {
            power = Math.signum(power) * 0.1;
        }

        // Tank-style turn
        drive.drive(
                -power,  // FL
                -power,  // FR
                power,  // BL
                -power   // BR (adjust if motors flipped)
        );
    }


    public boolean isFinishedThetaIMU() {
        double error = angleError(targetTheta, getHeading());
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
            case DRIVE_TURN_RIGHT: handleDriveTurn(true); break;
            case DRIVE_TURN_LEFT: handleDriveTurn(false); break;
            case DRIVE_STRAFE: handleStrafeState(); break;
            case AUTO_TURN: handleAutoTurnState(); break;
            case DRIVE_FORWARD_TIME: handleDriveTimeState(); break;
            case DRIVE_STRAFE_TIME: handleStrafeTimeState(); break;
            case DRIVE_BACK_TIME: handleDriveBackTimeState(); break;
            case DRIVE_STRAFE_BACK_TIME: handleStrafeBackTimeState(); break;
            case DRIVE_TURN_LEFT_TIME:
            case DRIVE_TURN_RIGHT_TIME:
                handleDriveTurnIMU(); break;
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

    private double clamp(double val) {
        return Math.max(-0.6, Math.min(0.6, val));
    }
}