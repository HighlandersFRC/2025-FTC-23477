package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.TAG_HEIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Commands.CommandShoot;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;
import org.firstinspires.ftc.teamcode.Tools.Limelight;


@TeleOp
public class motoeszdfguhijop extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        ShooterState shooter = new ShooterState("Imashootu");
        shooter.init(hardwareMap);


        waitForStart();
        while (opModeIsActive()) {

           shooter.periodic();

           shooter.shooterMotor.setPower(1);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target RPM", shooter.getTargetRPM());
            packet.put("Current Distance", Limelight.getDistance(TAG_HEIGHT));
            packet.put("CurrentRPM", shooter.computeRPM());
            packet.put("Feeding", shooter.isAtTargetVelocity());
            packet.put("FeedingStable", shooter.isAtTargetVelocityStable());
            packet.put("WhyFeed?", new CommandShoot(shooter, Limelight.getDistance(TAG_HEIGHT), 1000).isFinished());
            packet.put("distance", Limelight.getDistance(TAG_HEIGHT));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}