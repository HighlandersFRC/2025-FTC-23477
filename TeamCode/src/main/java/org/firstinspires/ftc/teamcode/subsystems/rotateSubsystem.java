package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;


public class rotateSubsystem extends Subsystem {
    private rotateStates wantedState = rotateStates.IDLE;
    private rotateStates currentState = rotateStates.IDLE;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    IMU imu;
    SparkFunOTOS mouse;
    PID cntclkpid;
    PID clkpid;
    PID forward_pid;
    double clkerror;
    double cntclkerror;
    double forward_error;
    double backward_error;
    double forward_distance;
    double backward_distance;
    double distance_traveled_x;
    double distance_traveled_y;
    double heading;

    public rotateSubsystem(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        leftFront = hardwareMap.dcMotor.get("left_front");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack = hardwareMap.dcMotor.get("left_back");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.dcMotor.get("right_front");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack = hardwareMap.dcMotor.get("right_back");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");

        SparkFunOTOS.Pose2D pos = mouse.getPosition();
        distance_traveled_x = pos.x;
        distance_traveled_y = pos.y;
        heading = pos.h;


        cntclkpid = new PID(0.05,0.01,0);
        clkpid = new PID(0.04, 0, 0.02);
        forward_pid = new PID(0.05, 0,0.01);

    }

    public enum rotateStates {
        CLOCKWISE_TURN,
        COUNTERCLOCKWISE_TURN,
        DRIVE_FORWARD_INCHES,
        DRIVE_BACKWARD_INCHES,
        IDLE
    }

    public void setWantedState(rotateStates rotateState) {
        this.wantedState = rotateState;
    }

    private void handleStateTransitions() {
        switch (wantedState) {
            case IDLE:
                setWantedState(rotateStates.IDLE);
                break;
            case CLOCKWISE_TURN:
                setWantedState(rotateStates.CLOCKWISE_TURN);
                break;
            case COUNTERCLOCKWISE_TURN:
                setWantedState(rotateStates.COUNTERCLOCKWISE_TURN);
                break;
            case DRIVE_FORWARD_INCHES:
                setWantedState(rotateStates.DRIVE_FORWARD_INCHES);
                break;
            case DRIVE_BACKWARD_INCHES:
                setWantedState(rotateStates.DRIVE_BACKWARD_INCHES);
                break;
        }

        currentState = wantedState;
    }

    private void handleIdleState() {
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    private void handleClockwiseState() {
        double yawAngle = heading;
        double target = -90;
        clkerror = yawAngle - target;
        double motor_power = clkpid.updatePID(clkerror);

        leftBack.setPower(-motor_power);
        leftFront.setPower(-motor_power);
        rightBack.setPower(motor_power);
        rightFront.setPower(motor_power);
    }

    private void handleCounterClockwiseState() {
        double yawAngle = heading;
        double target = 90;
        cntclkerror = target - yawAngle;
        double motor_power = cntclkpid.updatePID(cntclkerror);

        leftBack.setPower(motor_power);
        leftFront.setPower(motor_power);
        rightBack.setPower(-motor_power);
        rightFront.setPower(-motor_power);
    }

    private void handleDriveForwardInchesState(double distance_inches) {
        forward_error = distance_inches- distance_traveled_y;
        double motor_power = forward_pid.updatePID(forward_error);

        leftFront.setPower(motor_power);
        leftBack.setPower(motor_power);
        rightFront.setPower(motor_power);
        rightBack.setPower(motor_power);
    }

    private void handleDriveBackwardInchesState(double distance_inches) {
        backward_error = distance_inches-Math.abs(distance_traveled_y);
        double motor_power = forward_pid.updatePID(backward_error);

        leftFront.setPower(-motor_power);
        leftBack.setPower(-motor_power);
        rightFront.setPower(-motor_power);
        rightBack.setPower(-motor_power);
    }

    public void setForwardDistance(double distance) {
        this.forward_distance = distance;
    }

    public void setBackwardDistance(double distance) {
        this.backward_distance = distance;
    }

    public double get_x_traveled() {
        return distance_traveled_x;
    }

    public double get_y_traveled() {
        return distance_traveled_y;
    }

    public double get_current_heading() {
        return heading;
    }

    public boolean isFinishedClockwise() {
        return clkerror <= -1;
    }

    public boolean isFinishedCounterClockwise() {
        return cntclkerror <= 1;
    }

    public boolean isFinishedForward() {
        return forward_error <= 1.59*0.075;
    }

    public boolean isFinishedBackward() {
        return  backward_error <= -(1.59*0.075);
    }
    @Override
    public void periodic() {
        handleStateTransitions();
        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;
            case CLOCKWISE_TURN:
                handleClockwiseState();
                break;
            case COUNTERCLOCKWISE_TURN:
                handleCounterClockwiseState();
                break;
            case DRIVE_FORWARD_INCHES:
                handleDriveForwardInchesState(forward_distance);
                break;
            case DRIVE_BACKWARD_INCHES:
                handleDriveBackwardInchesState(backward_distance);
                break;
        }
    }
}
