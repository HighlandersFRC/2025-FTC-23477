package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;


@TeleOp
public class NewTestVot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        IndexerState indexerState = new IndexerState("indexer");
        ShooterState shooterState = new ShooterState("shooter");
        QueueState queueState = new QueueState("queue");
        Drive drive = new Drive("drive", hardwareMap);
        IntakeState intakeState = new IntakeState("intake");

        indexerState.init(hardwareMap);
        shooterState.init(hardwareMap);
        queueState.init(hardwareMap);
        intakeState.init(hardwareMap);
        Mouse.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {


            indexerState.periodic();
            shooterState.periodic();
            queueState.periodic();
            intakeState.periodic();


           if (gamepad1.a) {
               queueState.setWantedState(QueueState.QUEUE_STATE.QUEUE);
           } else if (gamepad1.b) {
               queueState.setWantedState(QueueState.QUEUE_STATE.REMOVE);
           } else {
               queueState.setWantedState(QueueState.QUEUE_STATE.DEFAULT);
           }


           if (gamepad1.x) {
               indexerState.setWantedState(IndexerState.INDEXER_STATE.INDEX);
           } else if (gamepad1.y) {
               indexerState.setWantedState(IndexerState.INDEXER_STATE.REMOVE);
           } else {
               indexerState.setWantedState(IndexerState.INDEXER_STATE.DEFAULT);
           }


           if (gamepad1.right_bumper) {
              shooterState.setTargetRPMFromDistance(Limelight.getDistance());
              shooterState.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
           } else if (gamepad1.left_bumper) {
               shooterState.setWantedState(ShooterState.SHOOTER_STATE.REMOVE);
           } else {
               shooterState.setWantedState(ShooterState.SHOOTER_STATE.IDLE);
           }

           if (gamepad1.left_trigger > 0 ) {
               intakeState.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
           } else if (gamepad1.right_trigger>0) {
               intakeState.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
           } else {
               intakeState.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
           }


            drive.FieldCentric(gamepad1);


            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", shooterState.getTargetRPM());
            packet.put("Current Distance", Limelight.getDistance());
            packet.put("CurrentRPM", shooterState.computeRPM());
            packet.put("Feeding", shooterState.isAtTargetVelocity());
            packet.put("FeedingStable", shooterState.isAtTargetVelocityStable());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}