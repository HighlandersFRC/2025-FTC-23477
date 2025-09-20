package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewVector;

@Autonomous
public class VectorAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive("drive", hardwareMap);

        double targetX = 0.0;
        double targetY = 1.0;
        double targetTheta = 0;

        waitForStart();


        double tolerancePos = 0.05;
        double toleranceTheta = Math.toRadians(2);

        while (opModeIsActive()) {
            double dx = targetX - Mouse.getX();
            double dy = targetY - Mouse.getY();
            double dTheta = targetTheta - Mouse.getTheta();

            if (Math.abs(dx) < tolerancePos && Math.abs(dy) < tolerancePos && Math.abs(dTheta) < toleranceTheta) {
                drive.stop();
            }

            drive.NewAutoDrive(new NewVector(dx, dy, dTheta));

            telemetry.addData("Target", "(%.2f, %.2f, %.1f°)", targetX, targetY, Math.toDegrees(targetTheta));
            telemetry.addData("Current", "(%.2f, %.2f, %.1f°)", Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.update();
        }
    }
}
