
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.AprilTagState;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterState;


public class NewRobot {

    // Instance variables for subsystems
    public Drive drive;
    public ShooterState shooterStates;
    public IntakeState intakeStates;
    public AprilTagState aprilTagState;

    public NewRobot(HardwareMap hardwareMap) {
        this.shooterStates = new ShooterState("shooter");
        this.intakeStates = new IntakeState("intake");
        this.aprilTagState = new AprilTagState("aprilTagState");
    }

    public void run() {

    }

    // Initialize hardware for all subsystems
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterStates.init(hardwareMap);
        intakeStates.init(hardwareMap);
        aprilTagState.init(hardwareMap);
    }

}
