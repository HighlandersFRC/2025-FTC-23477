package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class V2robot extends LinearOpMode {

    IntakeState intakeStates = new IntakeState("aashrithStates");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {

        Drive drive = new Drive("drive", hardwareMap);

        intakeStates.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.intakeStates = intakeStates;
        scheduler.setNewRobot(robot);

        if (gamepad1.right_trigger > 0) {
            intakeStates.setWantedState(IntakeState.INTAKE_STATE.INTAKE);
        } else if (gamepad1.left_trigger > 0) {
            intakeStates.setWantedState(IntakeState.INTAKE_STATE.OUTTAKE);
        } else {
            intakeStates.setWantedState(IntakeState.INTAKE_STATE.DEFAULT);
        }

        drive.FieldCentric(gamepad1);

        telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
        telemetry.addData("Controls", "D-pad Up=Enable Auto, D-pad Down=Manual");
        telemetry.update();
    }
}
