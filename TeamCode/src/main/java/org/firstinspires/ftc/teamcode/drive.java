package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.commands.commandIndex;
import org.firstinspires.ftc.teamcode.commands.commandScheduler;
import org.firstinspires.ftc.teamcode.commands.commandShoot;
import org.firstinspires.ftc.teamcode.subsystems.cameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.rotateSubsystem;
import org.firstinspires.ftc.teamcode.commands.commandAutoTarget;

@TeleOp
public class drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        rotateSubsystem drivetrain = new rotateSubsystem("drive");
        drivetrain.init(hardwareMap);

        intakeSubsystem intake = new intakeSubsystem("intake");
        intake.init(hardwareMap);

        cameraSubsystem camera = new cameraSubsystem("camera");
        camera.init(hardwareMap, telemetry);

        commandScheduler scheduler = new commandScheduler();
        boolean wasShootPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            boolean shootPressed = gamepad1.left_bumper;

            if (shootPressed && !wasShootPressed) {
                scheduler.schedule(
                        new ParallelCommandGroup(
                                scheduler,
                                Parameters.ALL,
                                new commandShoot(hardwareMap, 10),
                                new commandIndex(hardwareMap, 10)
                        )
                );
            }
            wasShootPressed = shootPressed;

            if (gamepad1.right_bumper) {
                intake.setWantedState(intakeSubsystem.states.OUTAKE);
            } else if (gamepad1.right_trigger > 0.1) {
                intake.setWantedState(intakeSubsystem.states.INTAKE);
            } else {
                intake.setWantedState(intakeSubsystem.states.IDLE);
            }

            if (gamepad1.a) {
                scheduler.schedule(new commandAutoTarget(hardwareMap, telemetry));
            }

            drivetrain.botCentricDrive(gamepad1);
            camera.periodic();
            intake.periodic();
            scheduler.run();
        }
    }
}
