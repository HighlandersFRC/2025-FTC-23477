package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.LineNumberReader;
@TeleOp
public class TurnMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     waitForStart();

     while (opModeIsActive()) {

         if (gamepad1.a) {
             motor1.setPower(-1);
         } else {
             motor1.setPower(0);
         }

     }
    }

}
