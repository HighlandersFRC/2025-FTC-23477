package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandAutoTarget;
import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.ConditionalCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;


@TeleOp
public class CommandTestAveVERYBADDONTUSE extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    CommandScheduler scheduler = new CommandScheduler();



    @Override
    public void runOpMode() throws InterruptedException {

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        CommandScheduler scheduler = new CommandScheduler();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                scheduler.schedule(new CommandAutoTarget());
                scheduler.run();

            }else{
                if (gamepad1.right_trigger > 0) {
                    scheduler.schedule(new CommandIntake(robot.intakeStates, 150));
                } else if (gamepad1.right_bumper) {
                    scheduler.schedule(
                            new ConditionalCommand(
                                    new ParallelCommandGroup(
                                            scheduler, Parameters.ALL,
                                            new CommandShoot(robot.shooterStates, 5000, 10000, 2500, true),
                                            new CommandIntake(robot.intakeStates, 2500)
                                    ),
                                    new CommandShoot(robot.shooterStates, 5000, 10000, 10000, true),
                                    () -> robot.shooterStates.isAtTargetVelocity()
                            )
                    );

                } else if (gamepad1.left_trigger > 0){
                    scheduler.schedule(new CommandOuttake(robot.intakeStates, 1000));
                }

                scheduler.run();

                drive.FieldCentric(gamepad1);
            }
        }
    }
}

