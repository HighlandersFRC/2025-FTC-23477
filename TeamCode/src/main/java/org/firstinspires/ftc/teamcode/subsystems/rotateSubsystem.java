package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

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
    PID cntclkpid;
    PID clkpid;
    double clkerror;
    double cntclkerror;

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

        cntclkpid = new PID(0.05,0.01,0);
        clkpid = new PID(0.04, 0, 0.01);

    }

    public enum rotateStates {
        CLOCKWISE_TURN,
        COUNTERCLOCKWISE_TURN,
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
        double yawAngle = imu.getRobotYawPitchRollAngles().getYaw();
        double target = -90;
        clkerror = yawAngle - target;
        double motor_power = clkpid.updatePID(clkerror);

        leftBack.setPower(motor_power);
        leftFront.setPower(motor_power);
        rightBack.setPower(-motor_power);
        rightFront.setPower(-motor_power);
    }

    private void handleCounterClockwiseState() {
        double yawAngle = imu.getRobotYawPitchRollAngles().getYaw();
        double target = 90;
        cntclkerror = target - yawAngle;
        double motor_power = cntclkpid.updatePID(cntclkerror);

        leftBack.setPower(motor_power);
        leftFront.setPower(motor_power);
        rightBack.setPower(-motor_power);
        rightFront.setPower(-motor_power);
    }

    public boolean isFinishedClockwise() {
        return clkerror <= -1;
    }

    public boolean isFinishedCounterClockwise() {
        return cntclkerror <= 1;
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
        }
    }
}
