package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;


@TeleOp
public class ShootTesting extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        double distance = 0;

        ShooterState shooterState = new ShooterState("shoot");

        shooterState.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                for (int i = 0; gamepad1.a; i++) {
                    distance = i;
                }
            }

            shooterState.setTargetRPMFromDistance(distance);
            shooterState.setWantedState(ShooterState.SHOOTER_STATE.SHOOT);

        }
    }
}