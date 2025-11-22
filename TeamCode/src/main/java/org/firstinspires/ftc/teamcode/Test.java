package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandJammed;
import org.firstinspires.ftc.teamcode.Commands.CommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;

import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.CommandStopIntakeOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandStopShoot;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@TeleOp
public class Test extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    SequencerState sequencerState = new SequencerState("sequencerState");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("AprilTag Adjustment", "ENABLED");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update subsystems
            intakeStates.periodic();
            shooterState.periodic();
            sequencerState.periodic();

            // Intake control
            if (gamepad1.right_trigger > 0) {
                scheduler.schedule(new CommandIntake(robot.intakeStates, 1000));
            }
            // Shoot control
            else if (gamepad1.right_bumper) {
                scheduler.schedule(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        scheduler, Parameters.ALL,
                                        new CommandShoot(robot.shooterStates, 5500, 15000),
                                        new CommandSpinRight(robot.sequencerState, 5500)
                                ),
                                new CommandShoot(robot.shooterStates, 5500, 15000),
                                () -> robot.shooterStates.isAtTargetVelocity()
                        )
                );
            }
            // Outtake control
            else if (gamepad1.left_trigger > 0) {
                scheduler.schedule(new CommandOuttake(robot.intakeStates, 1000));
            }

            // Run command scheduler
            scheduler.run();

            // Drive control
            drive.FeildCentric(gamepad1);

            // Telemetry
            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Shooter Target RPM", "%.0f", shooterState.getCurrentTargetRPM());
            telemetry.addData("Controls", "D-pad Up=Enable Auto, D-pad Down=Manual");
            telemetry.update();
        }
    }
}