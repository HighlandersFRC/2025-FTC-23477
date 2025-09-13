package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tools.Constants.DegreesToEncoderTicks;
import static org.firstinspires.ftc.teamcode.Tools.Constants.getDegrees;
import static org.firstinspires.ftc.teamcode.Tools.Constants.pivotPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;

import org.firstinspires.ftc.teamcode.Tools.NewRobot;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Wait;
import org.firstinspires.ftc.teamcode.PathingTool.PathLoading;
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

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        FieldOfMerit.initialize(hardwareMap);
        Mouse.init(hardwareMap);
        Mouse.configureOtos();
        NewRobot robot = new NewRobot(hardwareMap);
        Drive drive = new Drive("drive", hardwareMap);
        drive.setPosition(1.442, 0.276, 0);

        PathLoading path1 = new PathLoading(hardwareMap.appContext, "DriveOneMeter.polarpath");
        CommandScheduler scheduler = new CommandScheduler();
        drive = new Drive("drive", hardwareMap);
        Peripherals peripherals = new Peripherals("peripherals");
        PolarPathFollower Test;


        waitForStart();
        try {
            Test = new PolarPathFollower(drive, peripherals, path1.getJsonPathData(), Constants.commandMap, Constants.conditionMap, scheduler);
            scheduler.schedule(new SequentialCommandGroup(scheduler,
                    Test
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
            packet.put("Result", pivotPID.getResult());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X", -robotY);
            telemetry.addData("Y", -robotX);
            telemetry.addData("Theta", robotTheta);
            telemetry.addData("Time", currentTime);
            telemetry.update();
        }
    }
}