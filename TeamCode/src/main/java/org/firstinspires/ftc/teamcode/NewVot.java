package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.IndexTest;
import static org.firstinspires.ftc.teamcode.Tools.Constants.DURATION_MS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.TAG_HEIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Shooter.CommandShoot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.json.JSONException;

@TeleOp
public class NewVot extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    IndexerState indexerState = new IndexerState("indexer");
    QueueState queueState = new QueueState("queue");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        indexerState.init(hardwareMap);
        queueState.init(hardwareMap);
        Limelight.init(hardwareMap);
        Mouse.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        robot.queueState = queueState;
        robot.indexerState = indexerState;
        scheduler.setNewRobot(robot);


        Drive drive = new Drive("drive", hardwareMap);


        waitForStart();
        Mouse.configureOtos();
        while (opModeIsActive()) {
            intakeStates.periodic();
            shooterState.periodic();
            indexerState.periodic();
            queueState.periodic();


            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        IndexTest(scheduler, robot, DURATION_MS)
                );
            }

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }

            if (gamepad1.right_trigger > 0) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
                indexerState.setWantedState(IndexerState.INDEXER_STATE.INDEX);
            } else if (gamepad1.left_trigger > 0) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
                indexerState.setWantedState(IndexerState.INDEXER_STATE.REMOVE);
            } else {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
                indexerState.setWantedState(IndexerState.INDEXER_STATE.DEFAULT);
            }

            drive.FieldCentric(gamepad1);

boolean isFeeding = robot.queueState.getPower() < 0;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", shooterState.getTargetRPM());
            packet.put("Current Distance", Limelight.getDistance(TAG_HEIGHT));
            packet.put("CurrentRPM", shooterState.computeRPM());
            packet.put("Feeding", robot.shooterStates.isAtTargetVelocity());
            packet.put("FeedingStable", robot.shooterStates.isAtTargetVelocityStable());
            packet.put("WhyFeed?", new CommandShoot(robot.shooterStates, Limelight.getDistance(TAG_HEIGHT)).isFinished());
            packet.put("QueuerPower", isFeeding);
            packet.put("distance", Limelight.getDistance(TAG_HEIGHT));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Target RPM", shooterState.getTargetRPM());
            telemetry.addData("Is Detected", Limelight.isDetected());
            telemetry.addData("Current Distance", Limelight.getDistance(TAG_HEIGHT));
            telemetry.addData("CurrentRPM", shooterState.computeRPM());
            telemetry.update();

            telemetry.update();
        }
    }
}