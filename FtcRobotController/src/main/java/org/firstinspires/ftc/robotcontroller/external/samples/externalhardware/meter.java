package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class meter extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //sets up PID
        PID pidrf = new PID(0.03, 0, 0);
        PID pidrb = new PID(0.03, 0.0, 0.0);
        PID pidlf = new PID(0.03, 0.0, 0.0);
        PID pidlb = new PID(0.03, 0.0, 0.0);

        //sets up motors
        DcMotor rightFront;
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor rightBack;
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor leftFront;
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor leftBack;
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //starts code
        waitForStart();
        while (opModeIsActive()) {

            //sets double values
            double leftPos1 = leftFront.getCurrentPosition();
            double rightPos1 = rightFront.getCurrentPosition();
            double leftPos2 = leftBack.getCurrentPosition();
            double rightPos2 = rightBack.getCurrentPosition();
            double leftPowerf = pidlf.update(leftPos1);
            double leftPowerb = pidlb.update(leftPos2);
            double rightPowerf = pidrf.update(rightPos1);
            double rightPowerb = pidrb.update(rightPos2);

            leftPowerf = Math.max (0.4, Math.min(0.4, leftPowerf));
            leftPowerb = Math.max (0.4, Math.min(0.4, leftPowerb));
            rightPowerf = Math.max (0.4, Math.min(0.4, rightPowerf));
            rightPowerb = Math.max (0.4, Math.min(0.4, rightPowerb));

            //sets target positions
            leftFront.setTargetPosition(4507);
            leftBack.setTargetPosition(-4507);
            rightFront.setTargetPosition(4507);
            rightBack.setTargetPosition(-4507);

            //sets motor powers
            leftFront.setPower(leftPowerf);
            leftBack.setPower(leftPowerb);
            rightFront.setPower(rightPowerf);
            rightBack.setPower(rightPowerb);
        }
    }

}