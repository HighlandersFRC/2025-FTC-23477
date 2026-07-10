package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class cameraSubsystem extends Subsystem {

    states wanted_state = states.IDLE;
    states current_state = states.IDLE;

    DcMotor leftFront, leftBack, rightFront, rightBack;

    Limelight3A camera;
    PID pid = new PID(0.05, 0, 0.02);

    LLResult result;

    double tx;

    public cameraSubsystem(String name) {super(name);}

    public enum states {
        IDLE,
        AUTO_TARGET
    }

    public void init (HardwareMap hardwareMap) {
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        camera = hardwareMap.get(Limelight3A.class, "limelight");
        camera.pipelineSwitch(0);
        camera.start();
    }

    public void setWantedState(states state) {wanted_state = state;}

    public void handleStateTransitions() {
        switch (wanted_state) {
            case IDLE:
                setWantedState(states.IDLE);
                break;
            case AUTO_TARGET:
                setWantedState(states.AUTO_TARGET);
                break;
        }

        current_state = wanted_state;
    }

    private void handleIdleState() {

    }

    private void handleAutoTargetState() {
        result = camera.getLatestResult();
        if (result.isValid() && result != null) {
            tx = result.getTx();

            double motor_power = pid.updatePID(tx);

            if (tx > 0) {
                leftFront.setPower(-motor_power);
                leftBack.setPower(-motor_power);
                rightFront.setPower(motor_power);
                rightBack.setPower(motor_power);
            }
            if (tx < 0) {
                leftFront.setPower(motor_power);
                leftBack.setPower(motor_power);
                rightFront.setPower(-motor_power);
                rightBack.setPower(-motor_power);
            }
        }
    }

    public LLResult getResult() {
        return camera.getLatestResult();
    }

    public double getTx() {
        LLResult new_result = camera.getLatestResult();

        return new_result.getTx();
    }

    public double getTy(LLResult new_result) {
        return new_result.getTy();
    }

    public double getTa(LLResult new_result) {
        return new_result.getTa();
    }

    public double getDistanceFromTag(double ta) {
        return 27.6763 + (294304700 - 27.6763)/(1 + Math.pow(ta/9.277728e-10,0.6980128));
    }

    public boolean isFinished() {
        return tx < 1 && tx > -1;
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        switch (current_state) {
            case AUTO_TARGET:
                handleAutoTargetState();
                break;
            case IDLE:
                handleIdleState();
                break;
        }
    }


}
