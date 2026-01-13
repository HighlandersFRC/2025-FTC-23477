package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Tools.PID;

@TeleOp
public class elevator extends LinearOpMode {

    DcMotor elevator = hardwareMap.get(DcMotor.class, "intakeMotor");
    PID elevatorpid = new PID(0.33331611, 0.005, 0.01);
    double motorpos = elevator.getCurrentPosition();
    int targetpos = 1936;
    double error = targetpos-motorpos;
    double power = elevatorpid.updatePID(error);

    @Override
    public void runOpMode() {

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            waitForStart();

            if (gamepad1.left_stick_y <= 1 && gamepad1.left_stick_y >= 0.2) {

                //up
                elevator.setTargetPosition(-targetpos);
                elevator.setPower(power);

                if (gamepad1.a) {
                    break;
                }
            } else if (gamepad1.left_stick_y <= -1 && gamepad1.left_stick_y >= -0.2) {

                //down
                elevator.setTargetPosition(targetpos);
                elevator.setPower(power);

                if (gamepad1.a) {
                    break;
                }
            }

            telemetry.addData("power", power);
            telemetry.addData("Current Position", motorpos);
            telemetry.addData("Controls", "Elevator direction depends on left stick direction");
            telemetry.update();

        }
    }
}
