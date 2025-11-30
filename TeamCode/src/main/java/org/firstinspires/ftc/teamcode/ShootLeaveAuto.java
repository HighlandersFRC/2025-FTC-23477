package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@Autonomous
public class ShootLeaveAuto extends LinearOpMode {

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
        double RPM = 3533.33; long duration = 10000;
        ConditionalCommand SHOOT = new ConditionalCommand(
                new ParallelCommandGroup(
                        scheduler, Parameters.ALL,
                        new Wait(2000),
                        new CommandShoot(robot.shooterStates, RPM, duration),
                        new CommandSpinRight(robot.sequencerState, duration),
                        new CommandIntake(robot.intakeStates, duration)
                ),
                new CommandShoot(robot.shooterStates, RPM, duration),
                () -> robot.shooterStates.isAtTargetVelocity()
        );

        ParallelCommandGroup INTAKE = new ParallelCommandGroup(
                scheduler,
                Parameters.ALL,
                new SequentialCommandGroup(scheduler,
                        new Wait(1000),
                        new CommandIntake(robot.intakeStates, 1000),
                        new CommandSpinRight(robot.sequencerState, 30000)
                )
        );

        double distance = 1;

        scheduler.schedule(
                new SequentialCommandGroup(
                        scheduler,
                        SHOOT
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
