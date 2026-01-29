package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;
import static org.firstinspires.ftc.teamcode.Tools.Constants.DURATION_MS;
import static org.firstinspires.ftc.teamcode.Tools.Constants.TAG_HEIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.json.JSONException;

@TeleOp
public class Robot extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    SequencerState sequencerState = new SequencerState("sequencerState");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);
        Limelight.init(hardwareMap);
        Mouse.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        scheduler.setNewRobot(robot);


        Drive drive = new Drive("drive", hardwareMap);


        waitForStart();
        Mouse.configureOtos();
        while (opModeIsActive()) {
            intakeStates.periodic();
            shooterState.periodic();
            sequencerState.periodic();


            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        SHOOT(scheduler, robot, DURATION_MS,false)
                );
            }

                scheduler.run();

            if (gamepad1.right_trigger > 0) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
            } else if (gamepad1.left_trigger > 0) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
            } else {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
            }

            drive.FieldCentric(gamepad1);

            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Target RPM", shooterState.getTargetRPM());
            telemetry.addData("Is Detected", Limelight.isDetected());
            telemetry.addData("Current Distance", Limelight.getDistance(TAG_HEIGHT));
            telemetry.update();

            telemetry.update();
        }
    }
}