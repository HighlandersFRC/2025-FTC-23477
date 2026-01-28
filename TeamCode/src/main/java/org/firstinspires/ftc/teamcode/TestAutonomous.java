package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.IndexTest;
import static org.firstinspires.ftc.teamcode.Tools.Constants.TAG_HEIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandDrive;
import org.firstinspires.ftc.teamcode.Commands.CommandDriveBackTime;
import org.firstinspires.ftc.teamcode.Commands.CommandIndex;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandStrafeLeft;
import org.firstinspires.ftc.teamcode.Commands.CommandStrafeRight;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnLeftTime;
import org.firstinspires.ftc.teamcode.Commands.CommandTurnRightTime;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@Autonomous
public class TestAutonomous extends LinearOpMode {

    DriveStates drive = new DriveStates("drive");
    ShooterState shooterState = new ShooterState("shooterStates");
    IntakeState intakeState = new IntakeState("intake");
    IndexerState indexer = new IndexerState("index");
    QueueState queueState = new QueueState("queue");
    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        shooterState.init(hardwareMap);
        intakeState.init(hardwareMap);
        indexer.init(hardwareMap);
        queueState.init(hardwareMap);

        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.driveStates = drive;
        robot.shooterStates = shooterState;
        robot.intakeStates = intakeState;
        robot.indexerState = indexer;
        robot.queueState = queueState;
        scheduler.setNewRobot(robot);



        Mouse.configureOtos();

        waitForStart();


        scheduler.schedule(
                 new CommandTurnRightTime(robot.driveStates, 45)
        );


        while (opModeIsActive()) {
            Mouse.update();


            scheduler.run();

            drive.periodic();
            intakeState.periodic();
            shooterState.periodic();
            indexer.periodic();
            queueState.periodic();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", shooterState.getTargetRPM());
            packet.put("CurrentRPM", shooterState.computeRPM());
            packet.put("Feeding", robot.shooterStates.isAtTargetVelocity());
            packet.put("FeedingStable", robot.shooterStates.isAtTargetVelocityStable());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
