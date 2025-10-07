package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewVector;
import org.firstinspires.ftc.teamcode.Tools.Vector;

@Autonomous
public class VectorAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive("drive", hardwareMap);

        double targetX = 1.0;
        double targetY = 0.0;
        double targetTheta = 0;

        waitForStart();


        while (opModeIsActive()) {
            double dx = targetX - Mouse.getX();
            double dy = targetY - Mouse.getY();
            drive.autoDrive(new Vector(dx, dy), targetTheta);

            telemetry.addData("Target", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("Current", "(%.2f, %.2f, %.1f°)", Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.update();
        }
    }
}
