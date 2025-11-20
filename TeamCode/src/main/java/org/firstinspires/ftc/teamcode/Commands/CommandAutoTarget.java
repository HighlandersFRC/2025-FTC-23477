package org.firstinspires.ftc.teamcode.Commands;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Camsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

import java.util.List;

@TeleOp
public class CommandAutoTarget implements Command {
    private final PID drivePID = new PID(0, 0, 0);

    String name = "camera";


    @Override
    public void start() {


    }

    @Override
    public void execute() {
        IntakeState intakeStates = new IntakeState("aashrithStates");
        ShooterState shooterState = new ShooterState("shooterState");
        DriveStates driveStates = new DriveStates("drive");
        CommandScheduler scheduler = new CommandScheduler();

        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        //initializes rightFront
        DcMotor right_front;
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes leftFront
        DcMotor left_front;
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes rightBack
        DcMotor right_back;
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializes leftBack
        DcMotor left_back;
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SparkFunOTOS mouse;
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        mouse.begin();
        mouse.resetTracking();
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);

        //starts the code


        //gets the latest result from limelight
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            //initializes doubles
            double tx = result.getTx();
            double ty = result.getTy();
            double bx = result.getBotpose().getPosition().x;
            double by = result.getBotpose().getPosition().y;
            double rot = result.getBotpose().getOrientation().getYaw();
            double xmo = mouse.getPosition().x;
            double ymo = mouse.getPosition().y;
            double botx = ((xmo + bx) / 2);
            double boty = ((ymo + by) / 2);
            double revfactor = 99.9;
            double mouserot = mouse.getAngularScalar();

            //initializes pids
            //PID pidmt = new PID(0.04, 0.0, 0.01);
            PID theta = new PID(1, 0.0, 0.02);

            //continues double initialization
            double rotxfactor = (tx / 99.9 / 1440);
            double tarpos = result.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition().x;
            double avgmotopos = (right_back.getCurrentPosition() + right_front.getCurrentPosition() + left_back.getCurrentPosition() + left_front.getCurrentPosition());
            double motorpowerdivid = (theta.updatePID(rotxfactor + mouserot) / 2);

            //initializes integers
            int error = 1;
            int motorposr1 = right_front.getCurrentPosition();
            int motorposl1 = left_front.getCurrentPosition();
            int motorposr2 = right_back.getCurrentPosition();
            int motorposl2 = left_back.getCurrentPosition();
            int id = result.getFiducialResults().get(0).getFiducialId();
            int targetCount = fiducials.size();

            //initializes strings
            String family = result.getFiducialResults().get(0).getFamily();
            String curpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseFieldSpace());
            String robtarpos = String.valueOf(result.getFiducialResults().get(0).getRobotPoseTargetSpace());

            // initializes just ONE MORE double
            double power = motorpowerdivid / 2;

            if (tx < -error && id == 24) {
                //turn left
                right_front.setPower(power);
                left_front.setPower(power);
                right_back.setPower(-power);
                left_back.setPower(-power);
            } else if (tx > error && id == 24) {
                //turn right
                right_front.setPower(-power);
                left_front.setPower(-power);
                right_back.setPower(power);
                left_back.setPower(power);
            }

            if (targetCount == 0) {
                //no tag detected
                right_front.setPower(0);
                left_front.setPower(0);
                right_back.setPower(0);
                left_back.setPower(0);
            } else if (tx <= error && id == 24) {
                //completely localized
                telemetry.addData("status", "localized");
                telemetry.update();
                right_front.setPower(0);
                left_front.setPower(0);
                right_back.setPower(0);
                left_back.setPower(0);
            } else {
                //sus action occurred
                sleep(0);
            }

            System.out.println(tarpos);
        }
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Subsystem getRequiredSubsystem() {

        return new Camsystem("subsystem");
    }
}