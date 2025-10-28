package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;

import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.Vector;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo sequencer = hardwareMap.get(Servo.class, "sequencer");

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a) {
                sequencer.setPosition(1);
            } else if (gamepad1.b) {
                sequencer.setPosition(-1);
            }
            telemetry.addData("Current", "(%.2f, %.2f, %.1f°)", Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.update();
        }
    }
}
