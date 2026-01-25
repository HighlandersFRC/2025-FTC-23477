package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Tools.Constants.SHOOT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.DriveStates;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.SequencerState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;

import org.firstinspires.ftc.teamcode.Tools.Limelight;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.NewRobot;

@TeleOp
public class FollowTag extends LinearOpMode {
    DriveStates driveStates = new DriveStates("drivestates");
    CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void runOpMode() throws InterruptedException {
        driveStates.init(hardwareMap);

        Limelight.init(hardwareMap);

        NewRobot robot = new NewRobot(hardwareMap);
        robot.driveStates = driveStates;
        scheduler.setNewRobot(robot);


        Drive drive = new Drive("drive", hardwareMap);


        waitForStart();
        Mouse.configureOtos();
        while (opModeIsActive()) {
            // Update subsystems
            driveStates.periodic();

            driveStates.setWantedState(DriveStates.DRIVE_STATE.AUTO_TURN);

            // Telemetry
            telemetry.addData("Position", "(%.2f, %.2f, %.1f°)",
                    Mouse.getX(), Mouse.getY(), Math.toDegrees(Mouse.getTheta()));
            telemetry.addData("Distance", Limelight.getDistance(
                    0.762
            ));
            telemetry.addData("Port LL", Limelight.isConnected());
            telemetry.addData("Is Detected", Limelight.isDetected());
            telemetry.update();

            telemetry.update();
        }
    }
}