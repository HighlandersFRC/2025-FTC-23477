package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandJammed;
import org.firstinspires.ftc.teamcode.Commands.CommandRangeTrack;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.CommandStopIntakeOuttake;
import org.firstinspires.ftc.teamcode.Commands.CommandStopShoot;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Tools.Parameters;

@TeleOp
public class Robot extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ShooterState shooterStates = new ShooterState("shooter");
    IntakeState intakeStates = new IntakeState("Intake");
    AprilTagState aprilTagStates = new AprilTagState("aprilTagState");
    Drive drive = new Drive("drive", hardwareMap);
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        shooterStates.init(hardwareMap);
        intakeStates.init(hardwareMap);
        aprilTagStates.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.shooterStates = shooterStates;
        robot.intakeStates = intakeStates;
        robot.aprilTagState = aprilTagStates;
        scheduler.setNewRobot(robot);

        waitForStart();

        while (opModeIsActive()) {
            shooterStates.periodic();
            intakeStates.periodic();
            aprilTagStates.periodic();

            if (gamepad1.left_stick_button) {
                scheduler.schedule(new SequentialCommandGroup(scheduler, new CommandRangeTrack(robot.aprilTagState), new CommandShoot(robot.shooterStates, 3000, 2000, 3000, true)));
            } else if (gamepad1.right_trigger > 0){
                scheduler.schedule(new CommandIntake(robot.intakeStates, 5000));
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("X", Mouse.getX());
            packet.put("Y", Mouse.getY());
            packet.put("Theta", Mouse.getTheta());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X", Mouse.getX());
            telemetry.addData("Y", Mouse.getY());
            telemetry.addData("Theta", Mouse.getTheta());
            telemetry.update();
        }
    }
}
