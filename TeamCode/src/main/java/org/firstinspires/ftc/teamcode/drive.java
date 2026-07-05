package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    DcMotor left;
    DcMotor right;
    public void init (HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
    }

    public void drive(Gamepad gamepad) {
        double forward = gamepad.left_stick_y;
        double rotate = -gamepad.left_stick_x;

        double left_power = forward + rotate;
        double right_power = rotate - forward;

        left.setPower(left_power*0.7);
        right.setPower(right_power*0.7);
    }
}
