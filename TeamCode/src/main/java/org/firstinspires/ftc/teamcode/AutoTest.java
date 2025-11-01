package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.CommandTurn;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@Autonomous
public class AutoTest extends LinearOpMode {

    DriveStates drive = new DriveStates("drive");
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

        shooterState.setCameraConfig(
                14.0,  // Camera height in inches - ADJUST THIS
                30.0,  // AprilTag 24 height in inches - ADJUST THIS
                0.0   // Camera upward tilt angle in degrees - ADJUST THIS
        );
        shooterState.enableAprilTagAdjustment(true);

        drive.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);
        intakeState.init(hardwareMap);
        Mouse.configureOtos();

        waitForStart();

        // schedule the command ONCE
        scheduler.schedule(new SequentialCommandGroup(scheduler,
                new CommandTurn(robot.driveStates, 90),
                new CommandDrive(robot.driveStates, 0.5),
                new CommandIntake(robot.intakeStates, 1000)
        ));


        while (opModeIsActive()) {
            Mouse.update(); // keep updating OTOS data
            scheduler.run(); // process your commands
            drive.periodic(); // run the drive state machine
            intakeState.periodic();
            sequencerState.periodic();
            shooterState.periodic();

            telemetry.addData("MOuseX", Mouse.getX());
            telemetry.addData("MOuseTheta", Mouse.getTheta());
            telemetry.update();
        }
    }
}
