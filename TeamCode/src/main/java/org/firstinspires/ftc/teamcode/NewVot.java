package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.IndexTest;
import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;
import static org.firstinspires.ftc.teamcode.Tools.Constants.durationMs;
import static org.firstinspires.ftc.teamcode.Tools.Constants.tagHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class NewVot extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    SequencerState sequencerState = new SequencerState("sequencerState");
    IndexerState indexerState = new IndexerState("indexer");
    QueueState queueState = new QueueState("queue");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);
        indexerState.init(hardwareMap);
        queueState.init(hardwareMap);
        Limelight.init(hardwareMap);
        Mouse.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        robot.queueState = queueState;
        robot.indexerState = indexerState;
        scheduler.setNewRobot(robot);


        Drive drive = new Drive("drive", hardwareMap);


        waitForStart();
        Mouse.configureOtos();
        while (opModeIsActive()) {
            intakeStates.periodic();
            shooterState.periodic();
            sequencerState.periodic();
            indexerState.periodic();
            queueState.periodic();


            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        IndexTest(scheduler, robot, durationMs)
                );
            }

            scheduler.run();

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

            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Target RPM", shooterState.getTargetRPM());
            telemetry.addData("Is Detected", Limelight.isDetected());
            telemetry.addData("Current Distance", Limelight.getDistance(tagHeight));
            telemetry.addData("CurrentRPM", shooterState.computeRPM());
            telemetry.update();

            telemetry.update();
        }
    }
}