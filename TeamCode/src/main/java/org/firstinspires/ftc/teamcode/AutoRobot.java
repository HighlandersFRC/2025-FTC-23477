package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.CommandIntake;
import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.PathingTool.PolarPathFollower;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FieldOfMerit;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
@Autonomous
public class AutoRobot extends LinearOpMode {
    private FtcDashboard dashboard;
    private AprilTagState aprilTagState = new AprilTagState("aprilStates");
    private ShooterState shooterState = new ShooterState("shooterStates");
    private IntakeState intakeState = new IntakeState("intakeStates");
    @Override
    public void runOpMode() throws InterruptedException {

        intakeState.init(hardwareMap);
        shooterState.init(hardwareMap);
        aprilTagState.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        FieldOfMerit.initialize(hardwareMap);

        Mouse.init(hardwareMap);

        Mouse.configureOtos();

        CommandScheduler scheduler = new CommandScheduler();

        NewRobot robot = new NewRobot(hardwareMap);
        robot.aprilTagState = aprilTagState;
        robot.shooterStates = shooterState;
        robot.intakeStates = intakeState;
        scheduler.setNewRobot(robot);

        Drive drive = new Drive("drive", hardwareMap);

        drive.setPosition(0, 0, 0);

        PathLoading path1 = new PathLoading(hardwareMap.appContext, "");
        PathLoading path2 = new PathLoading(hardwareMap.appContext, "");

        drive = new Drive("drive", hardwareMap);
        Peripherals peripherals = new Peripherals("peripherals");

        PolarPathFollower AutoRobot1;
        PolarPathFollower AutoRobot2;


        waitForStart();
        try {

            AutoRobot1 = new PolarPathFollower(drive, peripherals, path1.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            AutoRobot2 = new PolarPathFollower(drive, peripherals, path2.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);

            scheduler.schedule(new SequentialCommandGroup(scheduler,
                    AutoRobot1,
                    new CommandIntake(robot.intakeStates),
                    AutoRobot2,
                    new CommandShoot(robot.shooterStates, 5)
            ));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        while (opModeIsActive()) {

            FinalPose.poseUpdate();

            scheduler.run();


            double robotX = FinalPose.x;
            double robotY = FinalPose.y;
            double robotTheta = FinalPose.Yaw;
            double currentTime = System.currentTimeMillis();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Robot X", -robotY);
            packet.put("Robot Y", -robotX);
            packet.put("Robot Theta", robotTheta);
            packet.put("Time", currentTime);
            packet.put("Target Angle", 150);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X", -robotY);
            telemetry.addData("Y", -robotX);
            telemetry.addData("Theta", robotTheta);
            telemetry.addData( "Time", currentTime);
            telemetry.update();
        }
    }
}