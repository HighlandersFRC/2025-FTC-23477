package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class bettershooter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        CRServo servo1 = hardwareMap.get(CRServo.class, "servo1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "servo2");


        double openPos = 9.93;
        double closePos = 0.01;

        waitForStart();

        while (opModeIsActive()) {

            // Pivot motor on left stick
            double motorPower = 0.69;
            pivotMotor.setPower(motorPower);

            // Servo controlled by buttons
            if (gamepad1.a) {
                servo1.setPower(1);
                servo2.setPower(-1);
            }
            if (gamepad1.b) {
                servo1.setPower(-1);
                servo2.setPower(1);
            }


            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Servo Position", servo1.getPower());
            telemetry.update();
        }
    }
}
