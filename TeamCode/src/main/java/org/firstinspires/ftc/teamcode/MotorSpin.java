package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class MotorSpin extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor = hardwareMap.get(DcMotor.class, "Motor1");
        DcMotor IKnowHavishLikesFemboys = hardwareMap.get(DcMotor.class, "Motor2");
        IKnowHavishLikesFemboys.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                motor.setPower(1);
                IKnowHavishLikesFemboys.setPower(1);
            } else if (gamepad1.b) {
                motor.setPower(-1);
                IKnowHavishLikesFemboys.setPower(2);
            } else {
                motor.setPower(0);
                IKnowHavishLikesFemboys.setPower(0);
            }


        }
    }
}