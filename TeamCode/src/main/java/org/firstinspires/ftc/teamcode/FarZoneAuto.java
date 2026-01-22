package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.Intake.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnRight;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Queuer.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.json.JSONException;

@Autonomous
public class FarZoneAuto extends LinearOpMode {
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


        double distance = 0.6;



        scheduler.schedule(
                new SequentialCommandGroup(
                        scheduler,
                        new CommandDrive(robot.driveStates, 1)
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