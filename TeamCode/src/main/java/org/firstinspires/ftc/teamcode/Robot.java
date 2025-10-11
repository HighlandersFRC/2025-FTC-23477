package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
@TeleOp
public class Robot extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    ShooterState shooterStates = new ShooterState("shooter");
    IntakeState intakeStates = new IntakeState("Intake");

    @Override
    public void runOpMode() throws InterruptedException {
        shooterStates.init(hardwareMap);
        intakeStates.init(hardwareMap);
        Drive driveSubsystem = new Drive("drive", hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            shooterStates.periodic();
            intakeStates.periodic();

            shooterStates.setWantedState(ShooterState.SHOOTER_STATE.DEFAULT);
            intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);


            driveSubsystem.FeildCentric(gamepad1);



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
