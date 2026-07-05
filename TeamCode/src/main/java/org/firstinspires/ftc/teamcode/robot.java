package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class robot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive();
        drive.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drive.drive(gamepad1);
        }
    }
}
