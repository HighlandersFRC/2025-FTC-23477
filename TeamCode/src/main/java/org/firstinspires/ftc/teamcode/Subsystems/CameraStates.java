package org.firstinspires.ftc.teamcode.Subsystems;



import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class CameraStates extends Subsystem {

    private CAMERA_STATES wantedSuperState = CAMERA_STATES.IDLE;
    private CAMERA_STATES currentSuperState = CAMERA_STATES.IDLE;
    private Drive drive;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Limelight3A limelight;
    private PID forwardPID;
    private PID strafePID;
    private PID turnPID;
    private LLResult result;
    private int id;
    private boolean automode;
    private boolean hasValidPose;
    private Pose3D botpose;
    private double x;
    private double z;
    private double tx;
    private double lastTx = 0;
    private double smoothTx;
    private double forwardPower;
    private double strafePower;
    private double turnPower;
    private double denominator;
    private double fl;
    private double bl;
    private double fr;
    private double br;
    public CameraStates(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        drive = new Drive("drive", hardwareMap);

        frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        backLeftMotor = hardwareMap.dcMotor.get("left_back");
        frontRightMotor = hardwareMap.dcMotor.get("right_front");
        backRightMotor = hardwareMap.dcMotor.get("right_back");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Robot Ready. Press PLAY.");
        telemetry.update();

        forwardPID = new PID(1,0,0.01);
        strafePID = new PID(1,0,0.01);
        turnPID = new PID(0.045, 0.0, 0.030);

        automode = false;
    }
    public void setWantedState(CAMERA_STATES cameraStates){
        wantedSuperState = cameraStates;
    }

    public enum CAMERA_STATES {
        DEFAULT,
        IDLE,
        AUTO_TARGET,
        DRIVE
    }

    private CAMERA_STATES handleStateTransitions() {
        switch (wantedSuperState) {
            case DEFAULT:
                currentSuperState = CAMERA_STATES.DEFAULT;
                break;
            case IDLE:
                currentSuperState = CAMERA_STATES.IDLE;
                break;
            case  AUTO_TARGET:
                currentSuperState = CAMERA_STATES.AUTO_TARGET;
                break;
            case DRIVE:
                currentSuperState = CAMERA_STATES.DRIVE;
                break;
        }
        return currentSuperState;
    }

    private void handleDefaultState() {
        drive.stop();
    }

    private void handleIdleState() {
        drive.stop();
        id = result.getFiducialResults().get(0).getFiducialId();

        telemetry.addData("id", id);
        telemetry.update();

    }
    private void handleAutoTargetState() {
        hasValidPose = result.isValid() && result != null;

        if (gamepad1.a) {
            automode = !automode;
            sleep(300);
        }

        if (automode && hasValidPose) {
            botpose = result.getBotpose();

            x = botpose.getPosition().x;
            z = botpose.getPosition().z;
            tx = result.getTx();

            smoothTx = 0.3 * lastTx + 0.7 * tx;
            lastTx = smoothTx;

            forwardPower = -forwardPID.updatePID(z);
            strafePower = strafePID.updatePID(x);
            turnPower = turnPID.updatePID(smoothTx);

            if (z > 0.4) {
                forwardPower *= 0.5;
                turnPower *= 0.5;
            }

            forwardPower = Math.max(-0.8, Math.min(0.8, forwardPower));
            strafePower  = Math.max(-0.6, Math.min(0.6, strafePower));
            turnPower    = Math.max(-0.5, Math.min(0.5, turnPower));

            denominator = Math.max(Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
            fl = (-forwardPower + strafePower + turnPower) / denominator;
            bl = (-forwardPower - strafePower + turnPower) / denominator;
            fr = (-forwardPower - strafePower - turnPower) / denominator;
            br = (-forwardPower + strafePower - turnPower) / denominator;

            frontLeftMotor.setPower(-fl);
            frontRightMotor.setPower(-fr);
            backLeftMotor.setPower(-bl);
            frontRightMotor.setPower(-br);

            telemetry.addData("Mode", "AUTO - PID Active");
            telemetry.addData("TX (deg)", "%.2f", tx);
            telemetry.addData("ForwardPID", "%.2f", forwardPower);
            telemetry.addData("StrafePID", "%.2f", strafePower);
            telemetry.addData("TurnPID", "%.2f", turnPower);
        }
    }
    public void handleDriveState() {
        drive.FieldCentric(gamepad1);
        telemetry.addData("Mode", "MANUAL");
    }

    public boolean isFinished() {
        return !result.isValid();
    }
    @Override
    public void periodic() {
        handleStateTransitions();
        switch (currentSuperState) {
            case DEFAULT:
                handleDefaultState();
                break;
            case IDLE:
                handleIdleState();
                break;
            case AUTO_TARGET:
                handleAutoTargetState();
                break;
            case DRIVE:
                handleDriveState();
                break;
        }
    }

}