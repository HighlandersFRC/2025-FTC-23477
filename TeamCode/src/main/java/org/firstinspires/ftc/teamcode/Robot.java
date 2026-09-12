package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure.Superstates;

@TeleOp
public class Robot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Indexer indexer = new Indexer("Indexer");
        Intake intake = new Intake("Intake");
        Shooter shooter = new Shooter("Shooter");
        Superstructure superstructure = new Superstructure("System", intake, shooter, indexer);
        superstructure.init(hardwareMap);


        while(opModeIsActive()) {
            superstructure.setWantedState(Superstates.DEFAULT);

            if (gamepad1.right_trigger > 0) {
                superstructure.setWantedState(Superstates.SHOOT);
            }

            if (gamepad1.left_trigger > 0) {
                superstructure.setWantedState(Superstates.INTAKE);
            }

            if (gamepad1.left_bumper) {
                superstructure.setWantedState(Superstates.OUTTAKE);
            }
        }
    }
}
