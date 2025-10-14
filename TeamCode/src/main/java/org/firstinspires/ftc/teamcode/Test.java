package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;

public class Test extends LinearOpMode {
    ShooterState shooterStates = new ShooterState("aprilTagState");
    CommandScheduler scheduler = new CommandScheduler();
    @Override
    public void runOpMode() throws InterruptedException {
        shooterStates.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.shooterStates = shooterStates;
        scheduler.setNewRobot(robot);
        waitForStart();
        while (opModeIsActive()) {
            shooterStates.periodic();

            if (gamepad1.a) {
                shooterStates.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
            } else if (gamepad1.b) {
                scheduler.schedule(new CommandShoot(robot.shooterStates, 5));
            }
        }
    }

}
