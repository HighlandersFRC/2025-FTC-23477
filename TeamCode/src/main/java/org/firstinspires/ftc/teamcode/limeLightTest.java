package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandRangeTrack;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class limeLightTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagState aprilTagState = new AprilTagState("drive");
        IntakeState intakeState = new IntakeState("intake");
        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.aprilTagState = aprilTagState;
        robot.intakeStates = intakeState;
        scheduler.setNewRobot(robot);
        waitForStart();

        intakeState.init(hardwareMap);
        aprilTagState.init(hardwareMap);

        scheduler.schedule(new SequentialCommandGroup(
                scheduler,
                new CommandRangeTrack(robot.aprilTagState),
                new CommandIntake(robot.intakeStates, 1000)
        ));


        while (opModeIsActive()) {
            aprilTagState.periodic();
            intakeState.periodic();

            scheduler.run();

            telemetry.addData("X", Mouse.getX());
            telemetry.addData("Y", Mouse.getY());
            telemetry.addData("Theta", Mouse.getTheta());
            telemetry.update();
        }
    }
}
