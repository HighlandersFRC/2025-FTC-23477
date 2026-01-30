package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandIndex;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandQueue;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandWaitToShoot;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

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

            double distance = Limelight.getDistance();
            long duration = 5000;
            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        scheduler,
                                        Parameters.ANY,
                                        new CommandShoot(robot.shooterStates, distance, duration),
                                        new CommandQueue(robot.queueState, duration)
                                ),
                                new CommandWaitToShoot(robot.shooterStates, distance),
                                () -> robot.shooterStates.isAtTargetVelocity()
                        )
                );
            }

                scheduler.run();

//            if (gamepad1.right_trigger > 0) {
//                intakeStates.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
//                indexerState.setWantedState(IndexerState.INDEXER_STATE.INDEX);
//            } else if (gamepad1.left_trigger > 0) {
//                intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
//                indexerState.setWantedState(IndexerState.INDEXER_STATE.REMOVE);
//            }
//            } else {
//                intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
//                indexerState.setWantedState(IndexerState.INDEXER_STATE.DEFAULT);
//            }
        if (!scheduler.isSubsystemBusy(indexerState)) {
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
        }





            drive.FieldCentric(gamepad1);

            boolean isFeeding = robot.queueState.getPower() > 0;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", shooterState.getTargetRPM());
            packet.put("Current Distance", Limelight.getDistance());
            packet.put("CurrentRPM", shooterState.computeRPM());
            packet.put("Feeding", robot.shooterStates.isAtTargetVelocity());
            packet.put("FeedingStable", robot.shooterStates.isAtTargetVelocityStable());
            packet.put("WhyFeed?", new CommandShoot(robot.shooterStates, Limelight.getDistance(), 1000).isFinished());
            packet.put("QueuerPower", isFeeding);
            packet.put("distance", Limelight.getDistance());
            packet.put("Ticks", robot.shooterStates.getTicks());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


            telemetry.update();
        }
    }
}