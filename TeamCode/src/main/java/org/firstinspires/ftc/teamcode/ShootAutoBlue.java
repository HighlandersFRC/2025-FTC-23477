package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandStrafe;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnRight;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@Autonomous
public class ShootAutoBlue extends LinearOpMode {
    DriveStates drive = new DriveStates("driver");
    ShooterState shooterState = new ShooterState("shooterStates");
    SequencerState sequencerState = new SequencerState("sequncer");
    IntakeState intakeState = new IntakeState("intake");
    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.driveStates = drive;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        robot.intakeStates = intakeState;
        scheduler.setNewRobot(robot);

        robot.initialize(hardwareMap);

        Mouse.configureOtos();

        waitForStart();


        double distance = 0.7;




        long waitDuration = 100;
        ParallelCommandGroup INTAKE = new ParallelCommandGroup(
                scheduler,
                Parameters.ALL,
                new CommandDrive(robot.driveStates, distance), // forward 1 meter, Mouse.X reset internally
                new SequentialCommandGroup(scheduler,
                        new Wait(waitDuration),
                        new CommandIntake(robot.intakeStates, 1000)
                )
        );

        scheduler.schedule(
                new SequentialCommandGroup(
                        scheduler,
                        new CommandDrive(robot.driveStates, -0.95), //Tune This
                        SHOOT(scheduler, robot, true),
                        new CommandTurnRight(robot.driveStates, 60),
                        new Wait(0),
                        INTAKE,
                        new Wait(0),
                        new CommandDrive(robot.driveStates, -distance+0.1),
                        new Wait(0),
                        new CommandTurnLeft(robot.driveStates,-55),
                        new Wait(0),
                        new ParallelCommandGroup(
                                scheduler,
                                Parameters.ALL,
                                new CommandIntake(robot.intakeStates, 300),
                                SHOOT(scheduler, robot, true)
                        ),
                        new CommandStrafe(robot.driveStates, -0.5)
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