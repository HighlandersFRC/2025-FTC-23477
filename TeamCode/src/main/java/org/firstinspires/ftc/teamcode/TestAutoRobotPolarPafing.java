package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.Intake.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandGroups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.json.JSONException;

@Autonomous
public class TestAutoRobotPolarPafing extends LinearOpMode {
    ShooterState shooterState = new ShooterState("shooterStates");
    SequencerState sequencerState = new SequencerState("sequncer");
    IntakeState intakeState = new IntakeState("intake");
    @Override
    public void runOpMode() throws InterruptedException {
        FieldOfMerit.initialize(hardwareMap);
        Mouse.init(hardwareMap);

        Mouse.configureOtos();


        new PathLoading(hardwareMap.appContext, "Yipee.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        Drive drive = new Drive("drive", hardwareMap);
        PolarPathFollower moveToPosition;
        drive.setPosition(0,0,0);
        NewRobot robot = new NewRobot(hardwareMap);
        robot.shooterStates = shooterState;
        robot.sequencerState = sequencerState;
        robot.intakeStates = intakeState;
        scheduler.setNewRobot(robot);

       shooterState.init(hardwareMap);
       sequencerState.init(hardwareMap);
       intakeState.init(hardwareMap);

        try {
            moveToPosition = new PolarPathFollower(drive, PathLoading.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            scheduler.schedule(new SequentialCommandGroup(scheduler, moveToPosition, new CommandIntake(robot.intakeStates, 1000)));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        while (opModeIsActive()) {
            FinalPose.poseUpdate();

            try {
                scheduler.run();
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }


            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.yaw;

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Theta", robotTheta);
            telemetry.update();
        }
    }
}