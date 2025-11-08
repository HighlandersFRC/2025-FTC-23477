package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.PID;

import java.util.List;

@TeleOp
public class limeLightDriver extends LinearOpMode {

    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize intake and shooter states
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);

        // Initialize robot with intake and shooter states
        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        scheduler.setNewRobot(robot);

        // Initialize drive subsystem
        Drive drive = new Drive("drive", hardwareMap);

        // Initialize motors for vision control
        DcMotor right_front = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor left_front = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor right_back = hardwareMap.get(DcMotor.class, "right_back");
        DcMotor left_back = hardwareMap.get(DcMotor.class, "left_back");

        // Reset and prepare motors for encoder usage
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // PID controllers for vision alignment
        PID pidmt = new PID(0.04, 0.0, 0.0);
        PID pidrotx = new PID(0.04, 0.0, 0.0);
        PID pidroty = new PID(0.04, 0.0, 0.0);

        waitForStart();

        while (opModeIsActive()) {
            // Periodic updates for intake and shooter states
            intakeStates.periodic();
            shooterState.periodic();

            // Gamepad controls for commands (intake, shoot, outtake)
            if (gamepad1.right_trigger > 0) {
                scheduler.schedule(new CommandIntake(robot.intakeStates, 150));
            } else if (gamepad1.right_bumper) {
                scheduler.schedule(new ConditionalCommand(
                        new ParallelCommandGroup(
                                scheduler, Parameters.ALL,
                                new CommandShoot(robot.shooterStates, 5000, 10000, 2500, true),
                                new CommandIntake(robot.intakeStates, 2500)
                        ),
                        new CommandShoot(robot.shooterStates, 5000, 10000, 10000, true),
                        () -> robot.shooterStates.isAtTargetVelocity()
                ));
            } else if (gamepad1.left_trigger > 0) {
                scheduler.schedule(new CommandOuttake(robot.intakeStates, 1000));
            }

            // Run the scheduler
            scheduler.run();

            // Drive control - field centric drive with gamepad
            drive.FieldCentric(gamepad1);

            // Vision processing for turning based on Limelight input
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                double tx = result.getTx();
                double ty = result.getTy();
                double bx = result.getBotpose().getPosition().x;
                double by = result.getBotpose().getPosition().y;

                double avgmotopos = (right_back.getCurrentPosition() + right_front.getCurrentPosition() + left_back.getCurrentPosition() + left_front.getCurrentPosition());
                double motorpowerdivid = (pidmt.updatePID(avgmotopos) + pidrotx.updatePID(tx) + pidroty.updatePID(ty));
                double power = Math.max(-0.4, Math.abs(motorpowerdivid));

                int error = 1;
                int targetCount = fiducials.size();

                if (targetCount > 0) {
                    int id = fiducials.get(0).getFiducialId();

                    if (tx < -error && id == 24) {
                        right_front.setPower(-power);
                        left_front.setPower(-power);
                        right_back.setPower(power);
                        left_back.setPower(power);
                    } else if (tx > error && id == 24) {
                        right_front.setPower(power);
                        left_front.setPower(power);
                        right_back.setPower(-power);
                        left_back.setPower(-power);
                    } else {
                        // Stop motors if aligned
                        right_front.setPower(0);
                        left_front.setPower(0);
                        right_back.setPower(0);
                        left_back.setPower(0);
                    }
                } else {
                    // Stop motors if no target
                    right_front.setPower(0);
                    left_front.setPower(0);
                    right_back.setPower(0);
                    left_back.setPower(0);
                }

                // Telemetry for driver station
                telemetry.addData("yaw", result.getBotpose().getOrientation().getYaw());
                telemetry.addData("pitch", ty);
                telemetry.addData("coordinates", "x: " + bx + " y: " + by);
                telemetry.addData("tx", tx);
                telemetry.addData("id", (targetCount > 0) ? fiducials.get(0).getFiducialId() : "None");
                telemetry.addData("family", (targetCount > 0) ? fiducials.get(0).getFamily() : "None");
                telemetry.update();
            }
        }
    }
}
