package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
@TeleOp
public class FieldCentric extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Drive driveSubsystem = new Drive("drive", hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

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
