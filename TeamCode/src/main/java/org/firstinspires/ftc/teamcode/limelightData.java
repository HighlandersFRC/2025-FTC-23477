package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.cameraSubsystem;

@TeleOp
public class limelightData extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        cameraSubsystem camera = new cameraSubsystem("camera");
        camera.init(hardwareMap, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            LLResult result = camera.getResult();

            double ta = camera.getTa(result);
            double distance = camera.getDistanceFromTag(ta);

            telemetry.addData("Target Area", ta);
            telemetry.addData("Distance From Tag", distance);
            telemetry.update();
        }
    }
}
