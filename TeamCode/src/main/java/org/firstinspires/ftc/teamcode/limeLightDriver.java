package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnRight;

import java.util.List;

@TeleOp
public class limeLightDriver extends LinearOpMode {

    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    DriveStates driveStates = new DriveStates("drive");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        // Vision hardware
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        SparkFunOTOS mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
        mouse.begin();
        mouse.resetTracking();
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);

        // Initialize motors for manual vision-assisted driving
        DcMotor right_front = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor left_front = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor right_back = hardwareMap.get(DcMotor.class, "right_back");
        DcMotor left_back = hardwareMap.get(DcMotor.class, "left_back");

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Command and subsystem robot structure
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        driveStates.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    int targetCount = fiducials.size();

                    double tx = result.getTx();
                    int id = result.getFiducialResults().get(0).getFiducialId();
                    int error = 1;
                    PID theta = new PID(1.0, 0.0, 0.02);

                    double power = (theta.updatePID(tx));
                    double bx = result.getBotpose().getPosition().x;
                    double by = result.getBotpose().getPosition().y;

                    if (id == 24 && gamepad1.a) {

                            if (tx > error) {
                                scheduler.schedule(new CommandTurnLeft(driveStates, tx));
                                scheduler.run();
                            } else if (tx < -error) {
                                scheduler.schedule(new CommandTurnRight(driveStates, tx));
                            }
                            if (targetCount == 0 || tx == 0) {
                            right_front.setPower(0);
                            left_front.setPower(0);
                            right_back.setPower(0);
                            left_back.setPower(0);
                            }
                    }

                    telemetry.addData("tx", tx);
                    telemetry.addData("id", id);
                    telemetry.addData("coordinates", "x", bx, "y" , by);
                    telemetry.update();
                } else {
                    sleep(0);
                }

                if (gamepad1.right_trigger > 0) {
                    scheduler.schedule(new CommandIntake(robot.intakeStates, 150));
                } else if (gamepad1.right_bumper) {
                    scheduler.schedule(
                            new ConditionalCommand(
                                    new ParallelCommandGroup(
                                            scheduler, Parameters.ALL,
                                            new CommandShoot(robot.shooterStates, 5000, 10000, 2500, true),
                                            new CommandIntake(robot.intakeStates, 2500)
                                    ),
                                    new CommandShoot(robot.shooterStates, 5000, 10000, 10000, true),
                                    () -> robot.shooterStates.isAtTargetVelocity()
                            )
                    );
                } else if (gamepad1.left_trigger > 0) {
                    scheduler.schedule(new CommandOuttake(robot.intakeStates, 1000));
                }

                scheduler.run();
                drive.FieldCentric(gamepad1);

            }
        }
    }
}


