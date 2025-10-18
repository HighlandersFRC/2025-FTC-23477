package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandJammed;
import org.firstinspires.ftc.teamcode.Commands.CommandOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;

import org.firstinspires.ftc.teamcode.Commands.CommandStopIntakeOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandStopShoot;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@TeleOp
public class Test extends LinearOpMode {
    IntakeState intakeStates = new IntakeState("aashrithStates");
    ShooterState shooterState = new ShooterState("shooterState");
    CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        intakeStates.init(hardwareMap);
        shooterState.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        robot.shooterStates = shooterState;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            intakeStates.periodic();
            shooterState.periodic();

            if (gamepad1.right_trigger > 0) {
                scheduler.schedule(new CommandIntake(robot.intakeStates, 150));
            } else if (gamepad1.right_bumper) {
                scheduler.schedule(new ParallelCommandGroup(scheduler, Parameters.ANY, new CommandIntake(robot.intakeStates, 5000), new CommandShoot(robot.shooterStates, 5000)));
            } else if (gamepad1.left_trigger > 0){
                scheduler.schedule(new CommandOuttake(robot.intakeStates, 1000));
            }

            scheduler.run();

            drive.FeildCentric(gamepad1);
        }
    }

}
