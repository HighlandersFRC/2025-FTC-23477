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
public class ShootAuto extends LinearOpMode {
    DriveStates drive = new DriveStates("driver");
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

        double RPM = 5500; long duration = 15000;
        ConditionalCommand SHOOT = new ConditionalCommand(
                new ParallelCommandGroup(
                        scheduler, Parameters.ALL,
                        new CommandShoot(robot.shooterStates, RPM, duration),
                        new CommandSpinRight(robot.sequencerState, duration)
                ),
                new CommandShoot(robot.shooterStates, RPM, duration),
                () -> robot.shooterStates.isAtTargetVelocity()
        );

        double distance = 0.6;
        ParallelCommandGroup INTAKE = new ParallelCommandGroup(
                scheduler,
                Parameters.ALL,
                new CommandDrive(robot.driveStates, distance), // forward 1 meter, Mouse.X reset internally
                new SequentialCommandGroup(scheduler,
                new Wait(1000),
                new CommandIntake(robot.intakeStates, 1000)
                )
        );
        scheduler.schedule(
                new SequentialCommandGroup(
                        scheduler,
                        SHOOT,
                        new CommandTurnLeft(robot.driveStates, -45),
                        new Wait(0),
                        INTAKE,
                        new Wait(0),
                        new CommandDrive(robot.driveStates, -0.6),
                        new Wait(0),
                        new CommandTurnRight(robot.driveStates, 40),
                        SHOOT,
                        new CommandTurnLeft(robot.driveStates, -45),
                        new Wait(0),
                        new CommandDrive(robot.driveStates, 0.6)
                )
        );


        while (opModeIsActive()) {
            Mouse.update();

            scheduler.run();

            drive.periodic();
            intakeState.periodic();
            sequencerState.periodic();
            shooterState.periodic();

            telemetry.addData("MOuseX", Mouse.getX());
            telemetry.addData("MOuseTheta", Mouse.getTheta());
            telemetry.update();
        }
    }
}