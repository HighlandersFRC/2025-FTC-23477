package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnRight;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@Autonomous
public class AutoRobot extends LinearOpMode {

    DriveStates drive = new DriveStates("drive");
    ShooterState shooterState = new ShooterState("shooterStates");
    SequencerState sequencerState = new SequencerState("sequncer");
    IntakeState intakeState = new IntakeState("intake");
    AprilTagState aprilTagState = new AprilTagState("aprilTag");

    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.driveStates = drive;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        robot.intakeStates = intakeState;
        robot.aprilTagState = aprilTagState;
        scheduler.setNewRobot(robot);

        robot.initialize(hardwareMap);
        Mouse.configureOtos();

        waitForStart();

        // --- Schedule autonomous sequence ---
        scheduler.schedule(
                new SequentialCommandGroup(scheduler,
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        scheduler, Parameters.ALL,
                                        new CommandShoot(robot.shooterStates, 5500, 15000),
                                        new CommandSpinRight(robot.sequencerState, 5500)
                                ),
                                new CommandShoot(robot.shooterStates, 5500, 15000),
                                () -> robot.shooterStates.isAtTargetVelocity()
                        ),
                        new Wait(0),
                        new CommandTurnLeft(robot.driveStates, -135),   // positive 90
                        new Wait(0),
                        new ParallelCommandGroup(
                                scheduler,
                                Parameters.ALL,
                                    new CommandDrive(robot.driveStates, 0.4), // forward 1 meter, Mouse.X reset internally
                                    new CommandIntake(robot.intakeStates, 1000)
                        ),
                        new Wait(0),
                        new CommandDrive(robot.driveStates, -0.4),
                        new Wait(0),
                        new CommandTurnRight(robot.driveStates, 135),
                        new SequentialCommandGroup(scheduler,
                                new ConditionalCommand(
                                        new ParallelCommandGroup(
                                                scheduler, Parameters.ALL,
                                                new CommandShoot(robot.shooterStates, 5500, 15000),
                                                new CommandSpinRight(robot.sequencerState, 5500)
                                        ),
                                        new CommandShoot(robot.shooterStates, 5500, 15000),
                                        () -> robot.shooterStates.isAtTargetVelocity()
                                ),
                                new CommandTurnLeft(robot.driveStates, -135),   // positive 90
                                new Wait(0),
                                new CommandDrive(robot.driveStates, 1)
                )
        ));


        // --- Main loop ---
        while (opModeIsActive()) {
            Mouse.update();
            scheduler.run();
            drive.periodic();
            intakeState.periodic();
            sequencerState.periodic();
            shooterState.periodic();

            telemetry.addData("MouseX", Mouse.getX());
            telemetry.addData("MouseTheta", Mouse.getTheta());
            telemetry.update();
        }
    }
}