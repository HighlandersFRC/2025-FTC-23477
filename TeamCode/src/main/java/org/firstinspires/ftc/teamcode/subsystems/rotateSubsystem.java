package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class rotateSubsystem extends Subsystem {
    private static final double HEADING_TOLERANCE_DEGREES = 1.0;
    private static final double DISTANCE_TOLERANCE_INCHES = 0.25;
    private static final double MAX_TURN_POWER = 0.6;
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double STRAIGHT_HEADING_KP = 0.025;
    private static final double TELEOP_HEADING_KP = 0.02;
    private static final double TELEOP_MAX_HEADING_CORRECTION = 0.25;
    private static final double JOYSTICK_DEADBAND = 0.06;
    private static final double STRAFE_ADJUSTMENT = 1.1;

    private double clockwiseTargetDegrees = -90.0;
    private double counterClockwiseTargetDegrees = 90.0;

    private rotateStates wantedState = rotateStates.IDLE;
    private rotateStates currentState = rotateStates.IDLE;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private IMU imu;
    private SparkFunOTOS odometry;

    private PID clockwisePid;
    private PID counterClockwisePid;
    private PID drivePid;

    private double targetForwardDistanceInches;
    private double targetBackwardDistanceInches;
    private double startXInches;
    private double startYInches;
    private double startHeadingDegrees;
    private double xInches;
    private double yInches;
    private double headingDegrees;
    private double teleopHeadingTargetDegrees;
    private boolean teleopHeadingHoldEnabled;
    private double headingErrorDegrees;
    private double distanceErrorInches;

    public rotateSubsystem(String name) {
        super(name);
    }

    public enum rotateStates {
        CLOCKWISE_TURN,
        COUNTERCLOCKWISE_TURN,
        DRIVE_FORWARD_INCHES,
        DRIVE_BACKWARD_INCHES,
        IDLE
    }

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightBack = hardwareMap.dcMotor.get("right_back");

        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        odometry = hardwareMap.get(SparkFunOTOS.class, "mouse");
        updatePose();
        resetMotionStart();
        teleopHeadingTargetDegrees = headingDegrees;

        clockwisePid = createClockwisePid();
        counterClockwisePid = createCounterClockwisePid();
        drivePid = createDrivePid();
    }

    public void setWantedState(rotateStates state) {
        if (state != currentState) {
            resetMotionStart();
            headingErrorDegrees = 0.0;
            distanceErrorInches = 0.0;
            drivePid = createDrivePid();
        }

        wantedState = state;
    }

    public void setForwardDistance(double distanceInches) {
        targetForwardDistanceInches = Math.abs(distanceInches);
    }

    public void setBackwardDistance(double distanceInches) {
        targetBackwardDistanceInches = Math.abs(distanceInches);
    }

    public void botCentricDrive(Gamepad gamepad) {
        updatePose();

        double y = -gamepad.left_stick_y;
        double x = gamepad.right_stick_x * STRAFE_ADJUSTMENT;
        double rotation = -gamepad.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1.0);

        double frontLeftPower = (-y + x - rotation) / denominator;
        double backLeftPower = (-y + x + rotation) / denominator;
        double frontRightPower = (y + x - rotation) / denominator;
        double backRightPower = (-y - x - rotation) / denominator;

        setDrivePower(-frontLeftPower, -backLeftPower, -frontRightPower, -backRightPower);
    }

    public double getXTraveled() {
        return xInches - startXInches;
    }

    public double getYTraveled() {
        return yInches - startYInches;
    }

    public double getCurrentHeading() {
        return headingDegrees;
    }

    public void setClockwiseAngle(double degrees) {
        clockwiseTargetDegrees = degrees;
    }

    public void setCounterclockwiseAngle(double degrees) {
        counterClockwiseTargetDegrees = degrees;
    }

    public boolean isFinishedClockwise() {
        return Math.abs(angleError(clockwiseTargetDegrees, headingDegrees)) <= HEADING_TOLERANCE_DEGREES;
    }

    public boolean isFinishedCounterClockwise() {
        return Math.abs(angleError(counterClockwiseTargetDegrees, headingDegrees)) <= HEADING_TOLERANCE_DEGREES;
    }

    public boolean isFinishedForward() {
        return Math.abs(targetForwardDistanceInches - getForwardDistanceTraveled()) <= DISTANCE_TOLERANCE_INCHES;
    }

    public boolean isFinishedBackward() {
        return Math.abs(targetBackwardDistanceInches - getBackwardDistanceTraveled()) <= DISTANCE_TOLERANCE_INCHES;
    }

    @Override
    public void periodic() {
        updatePose();
        currentState = wantedState;

        switch (currentState) {
            case CLOCKWISE_TURN:
                turnToHeading(clockwiseTargetDegrees, clockwisePid);
                break;
            case COUNTERCLOCKWISE_TURN:
                turnToHeading(counterClockwiseTargetDegrees, counterClockwisePid);
                break;
            case DRIVE_FORWARD_INCHES:
                driveToDistance(targetForwardDistanceInches, 1.0);
                break;
            case DRIVE_BACKWARD_INCHES:
                driveToDistance(targetBackwardDistanceInches, -1.0);
                break;
            case IDLE:
            default:
                stopDrive();
                break;
        }
    }

    private void updatePose() {
        SparkFunOTOS.Pose2D pose = odometry.getPosition();
        xInches = pose.x;
        yInches = pose.y;
        headingDegrees = pose.h;
    }

    private void resetMotionStart() {
        startXInches = xInches;
        startYInches = yInches;
        startHeadingDegrees = headingDegrees;
    }

    private void turnToHeading(double targetHeadingDegrees, PID pid) {
        headingErrorDegrees = angleError(targetHeadingDegrees, headingDegrees);
        double power = Range.clip(pid.updatePID(headingErrorDegrees), -MAX_TURN_POWER, MAX_TURN_POWER);

        setDrivePower(power, power, -power, -power);
    }

    private void driveToDistance(double targetDistanceInches, double direction) {
        double distanceTraveledInches = (yInches - startYInches) * direction;
        distanceErrorInches = targetDistanceInches - distanceTraveledInches;

        double drivePower = Range.clip(drivePid.updatePID(distanceErrorInches), -MAX_DRIVE_POWER, MAX_DRIVE_POWER)
                * direction;
        double headingCorrection = Range.clip(
                angleError(startHeadingDegrees, headingDegrees) * STRAIGHT_HEADING_KP,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );

        setDrivePower(
                drivePower + headingCorrection,
                drivePower + headingCorrection,
                drivePower - headingCorrection,
                drivePower - headingCorrection
        );
    }

    private double getForwardDistanceTraveled() {
        return yInches - startYInches;
    }

    private double getBackwardDistanceTraveled() {
        return startYInches - yInches;
    }

    private double angleError(double targetDegrees, double currentDegrees) {
        double error = targetDegrees - currentDegrees;

        while (error > 180.0) {
            error -= 360.0;
        }

        while (error <= -180.0) {
            error += 360.0;
        }

        return error;
    }

    private double scaleJoystick(double input) {
        if (Math.abs(input) < JOYSTICK_DEADBAND) {
            return 0.0;
        }

        return Math.copySign(input * input, input);
    }

    private PID createClockwisePid() {
        return new PID(0.04, 0.0, 0.02);
    }

    private PID createCounterClockwisePid() {
        return new PID(0.05, 0.01, 0.0);
    }

    private PID createDrivePid() {
        return new PID(0.05, 0.0, 0.01);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setRunMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
    }

    private void stopDrive() {
        setDrivePower(0.0, 0.0, 0.0, 0.0);
    }

    private void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {
        leftFront.setPower(frontLeft);
        leftBack.setPower(backLeft);
        rightFront.setPower(frontRight);
        rightBack.setPower(backRight);
    }
}
