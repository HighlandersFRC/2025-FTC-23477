package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Mouse;
@TeleOp
public class RobotSuperStructureOnly extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ShooterState shooterStates = new ShooterState("shooter");
    IntakeState intakeStates = new IntakeState("intake");

    @Override
    public void runOpMode() throws InterruptedException {
        shooterStates.init(hardwareMap);
        intakeStates.init(hardwareMap);
        Drive drive = new Drive("Drive", hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            shooterStates.periodic();
            intakeStates.periodic();


            if (gamepad1.right_trigger > 0) {
                shooterStates.setTargetRPM(6000);
                shooterStates.setTargetTicks(15000);
                shooterStates.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);
            } else if (gamepad1.left_trigger > 0) {
                shooterStates.setWantedState(ShooterState.SHOOTER_STATE.JAMMED);
            } else {
                shooterStates.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
            }

            if (gamepad1.right_bumper) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
            } else if (gamepad1.left_bumper) {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
            } else {
                intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
            }

            drive.FeildCentric(gamepad1);


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
