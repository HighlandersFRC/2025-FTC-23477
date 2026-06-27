package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.commands.commandIndex;
import org.firstinspires.ftc.teamcode.commands.commandScheduler;
import org.firstinspires.ftc.teamcode.commands.commandShoot;
import org.firstinspires.ftc.teamcode.subsystems.indexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.rotateSubsystem;

@TeleOp
public class drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        rotateSubsystem drive = new rotateSubsystem("subsystem");
        drive.init(hardwareMap);
        intakeSubsystem intake = new intakeSubsystem("intake");
        intake.init(hardwareMap);
        commandScheduler scheduler = new commandScheduler();

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                scheduler.schedule(
                        new ParallelCommandGroup(
                                scheduler,
                                Parameters.ALL,
                                new commandShoot(hardwareMap, 10),
                                new commandIndex(hardwareMap, 10)
                        )
                );
            }
            else if (gamepad1.right_trigger != 1) {
                intake.setWantedState(intakeSubsystem.states.INTAKE);
            }
            else if (gamepad1.right_bumper) {
                intake.setWantedState(intakeSubsystem.states.OUTAKE);
            }

            else {
                intake.setWantedState(intakeSubsystem.states.IDLE);
            }

            drive.botCentricDrive(gamepad1);
        }
    }
}
