package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

@TeleOp
public class ASTest extends OpMode {
    Drive drive;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Mouse.init(hardwareMap);
       drive = new Drive("drive", hardwareMap);
    }

    @Override
    public void loop() {
        Mouse.update();
        drive.FieldCentric(gamepad1);
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Pose x", Mouse.getXM());
        packet.put("Pose y", Mouse.getYM());
        packet.put("Pose heading", Math.toRadians(Mouse.getTheta()));

        packet.put("Pose heading (deg)", Mouse.getTheta());


        packet.put("Example Value", 42);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        telemetry.addData("Pose x", 12.0);
        telemetry.addData("Pose y", 24.0);
        telemetry.addData("Pose heading (deg)", Math.toDegrees(1.57));
        telemetry.update();
    }
}
