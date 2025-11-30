package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class CommandRightStrafe implements Command {

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;
    PID pid;
    double targetX;
    double power;
    DriveStates drive;

    @Override
    public void start() {

    }

    @Override
    public void execute() {
        pid = new PID(0.3, 0.02, 0.01);

        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetX = 1;
        power = pid.updatePID(targetX - Mouse.getX());

        leftBack.setPower(power);
        leftFront.setPower(-power);
        rightBack.setPower(-power);
        rightFront.setPower(power);
    }

    @Override
    public void end() {

    }

    @Override
    public boolean isFinished() {
        return targetX == Mouse.getX();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}
