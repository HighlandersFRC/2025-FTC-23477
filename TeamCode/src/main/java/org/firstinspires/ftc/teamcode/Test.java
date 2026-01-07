package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class Test extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    SequencerState sequencerState = new SequencerState("sequencerState");
    DriveStates driveStates = new DriveStates("drivestates");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);
        sequencerState.init(hardwareMap);
        driveStates.init(hardwareMap);

        Limelight.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        robot.driveStates = driveStates;
        scheduler.setNewRobot(robot);


        Drive drive = new Drive("drive", hardwareMap);


        waitForStart();
        Mouse.configureOtos();
        while (opModeIsActive()) {
            // Update subsystems
            intakeStates.periodic();
            shooterState.periodic();
            sequencerState.periodic();
            driveStates.periodic();


            if (gamepad1.right_bumper) {
                scheduler.schedule(
                        SHOOT(scheduler, robot, 3200,false)
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

            boolean isTouched = false;
            boolean lastTouchpadState = false;


            boolean currentTouchpadState = gamepad1.touchpad_finger_1;

            if (currentTouchpadState && !lastTouchpadState) {
                isTouched = !isTouched;
            }


            lastTouchpadState = currentTouchpadState;

            if (isTouched) {
                driveStates.setWantedState(DriveStates.DRIVE_STATE.AUTO_TURN);
            } else if (driveStates.isFinishedAutoTurnTheta()) {
                drive.FieldCentric(gamepad1);
            }


            // Telemetry
            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Target RPM", shooterState.getTargetRPM());
            telemetry.addData("Current RPM", shooterState.getCurrentRPM());
            telemetry.addData("Distance", Limelight.getDistance(
                    0.762
            ));
            telemetry.addData("Port LL", Limelight.isConnected());
            telemetry.addData("Is Detected", Limelight.isDetected());

            telemetry.update();

            telemetry.update();
        }
    }
}