package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Drive.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnAuto;
import org.firstinspires.ftc.teamcode.Commands.Drive.CommandTurnLeft;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Queuer.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.json.JSONException;

@Autonomous
public class LeaveAuto extends LinearOpMode {

    DriveStates drive = new DriveStates("drive");
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


        scheduler.schedule(
                new CommandDrive(robot.driveStates, 1)
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
