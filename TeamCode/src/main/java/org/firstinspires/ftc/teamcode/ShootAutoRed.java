package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;
import static org.firstinspires.ftc.teamcode.Tools.Constants.DURATION_MS_AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Drive.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.Intake.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandStrafe;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnAuto;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnRight;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.json.JSONException;

@Autonomous
public class ShootAutoRed extends LinearOpMode {
    DriveStates drive = new DriveStates("driver");
    ShooterState shooterState = new ShooterState("shooterStates");
    SequencerState sequencerState = new SequencerState("sequncer");
    IntakeState intakeState = new IntakeState("intake");
    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);
        intakeState.init(hardwareMap);

        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.driveStates = drive;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        robot.intakeStates = intakeState;
        scheduler.setNewRobot(robot);


        Mouse.configureOtos();

        waitForStart();


        double distance = 0.42;


        long waitDuration = 100;
        ParallelCommandGroup INTAKE = new ParallelCommandGroup(
                scheduler,
                Parameters.ALL,
                new CommandDrive(robot.driveStates, distance),
                new SequentialCommandGroup(scheduler,
                new Wait(waitDuration),
                new CommandIntake(robot.intakeStates, 1000)
                )
        );


        scheduler.schedule(
                new SequentialCommandGroup(
                        scheduler,
                        new Wait(0),
                        new CommandDrive(robot.driveStates, -0.6), //Tune This
                        new Wait(0),
                        new CommandTurnAuto(robot.driveStates),
                        SHOOT(scheduler, robot, DURATION_MS_AUTO, true),
                        new CommandTurnLeft(robot.driveStates, -45),
                        new Wait(0),
                        new CommandStrafe(robot.driveStates, 0.36),
                        new Wait(0),
                        INTAKE,
                        new Wait(0),
                        new CommandDrive(robot.driveStates, -distance-0.15),
                        new Wait(0),
                        new CommandTurnRight(robot.driveStates,45),
                        new Wait(0),
                        new CommandTurnAuto(robot.driveStates),
                        new Wait(0),
                        new CommandDrive(robot.driveStates, 0.3),
                        SHOOT(scheduler, robot, DURATION_MS_AUTO,true),
                        new CommandStrafe(robot.driveStates, 0.4)
                )
        );


        while (opModeIsActive()) {
            Mouse.update();

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }

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