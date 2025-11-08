package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@Autonomous
public class LeaveAuto extends LinearOpMode {

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

        shooterState.setCameraConfig(
                14.0,
                30.0,
                0.0
        );

        shooterState.enableAprilTagAdjustment(true);

        robot.initialize(hardwareMap);

        Mouse.configureOtos();

        waitForStart();


        scheduler.schedule(
                new CommandDrive(robot.driveStates, 1)
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
