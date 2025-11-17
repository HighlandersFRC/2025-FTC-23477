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

        double RPM = 3100;
        double distance = 0.67; /* "dswsweer" - Avery Stewart & Purple Artifact made in Thailand with minimal Scratches */

        long ShootDur = 2700;
        ConditionalCommand Shoot = new ConditionalCommand(
                new ParallelCommandGroup(
                        scheduler, Parameters.ALL,
                        new CommandShoot(robot.shooterStates, RPM, ShootDur),
                        new CommandSpinRight(robot.sequencerState, ShootDur)
                ),
                new CommandShoot(robot.shooterStates, RPM, ShootDur),
                () -> robot.shooterStates.isAtTargetVelocity()
        );

        ParallelCommandGroup Intake =  new ParallelCommandGroup(
                scheduler,
                Parameters.ALL,
                new CommandDrive(robot.driveStates, distance), // forward 1 meter, Mouse.X reset internally
                new CommandIntake(robot.intakeStates, 500)
        );

        scheduler.schedule(
                new SequentialCommandGroup(
                        scheduler,
                        Shoot,
                        new CommandTurnLeft(robot.driveStates, -55),
                        new Wait(0),
                        Intake,
                        new Wait(0),
                        new CommandDrive(robot.driveStates, -distance+0.1),
                        new Wait(0),
                        new CommandTurnRight(robot.driveStates, 55),
                        Shoot,
                        new CommandDrive(robot.driveStates, -1)
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