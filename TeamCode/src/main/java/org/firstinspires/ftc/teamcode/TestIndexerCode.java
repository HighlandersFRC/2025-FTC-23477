package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.IndexTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.QueueState;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.json.JSONException;

@TeleOp
public class TestIndexerCode extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("intake");
    IndexerState indexerState = new IndexerState("indexer");
    QueueState queueState = new QueueState("queue");
    ShooterState shooterState = new ShooterState("Ima shoot avery");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        indexerState.init(hardwareMap);
        queueState.init(hardwareMap);
        shooterState.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.queueState = queueState;
        robot.indexerState = indexerState;
        robot.shooterStates = shooterState;
        scheduler.setNewRobot(robot);



        waitForStart();
        while (opModeIsActive()) {
            intakeStates.periodic();
            indexerState.periodic();
            queueState.periodic();
            shooterState.periodic();


            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        IndexTest(scheduler, robot, 1000)
                );
            }

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
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




            telemetry.update();

            telemetry.update();
        }
    }
}