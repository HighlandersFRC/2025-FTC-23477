package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;

import org.firstinspires.ftc.teamcode.Commands.CommandSpinRight;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@TeleOp
public class Test extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    SequencerState sequencerState = new SequencerState("sequencerState");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intakeStates.periodic();
            shooterState.periodic();
            sequencerState.periodic();

            if (gamepad1.right_trigger > 0) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
            } else if (gamepad1.left_trigger > 0) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
            } else {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
            }


//            if (gamepad1.b) {
//                sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.SPIN_RIGHT);
//            } else if (gamepad1.a) {
//                sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.SPIN_LEFT);
//            } else {
//                sequencerState.setWantedState(SequencerState.SEQUENCER_STATE.DEFAULT);
//            }



            // double RPM = 2500;
            double RPM = 3200;

            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        new ConditionalCommand(
                                new ParallelCommandGroup(
                                        scheduler, Parameters.ANY,
                                        new CommandShoot(robot.shooterStates, RPM, 2500),
                                        new CommandSpinRight(robot.sequencerState, 2500)
                                ),
                                new CommandShoot(robot.shooterStates, RPM, 2500),
                                () -> robot.shooterStates.isAtTargetVelocity()
                        )
                );
            }

            scheduler.run();



            drive.FeildCentric(gamepad1);

            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Shooter Target RPM", "%.0f", shooterState.getCurrentTargetRPM());
            telemetry.addData("Shooter Current RPM", shooterState.getCurrentRPM());
            telemetry.addData("Shooter Is Finished" , shooterState.isAtTargetVelocity());
            telemetry.update();
        }
    }
}