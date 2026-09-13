package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Limelight extends Subsystem {
    Limelight3A limelight;
    public Limelight(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, Constants.InitInfo.Shooter.LIMELIGHT_NAME);
        limelight.start();
    }


    public LLResult getResult() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        return result;
    }

    // Mega tag 1
    public Pose2D getPose() {
        LLResult result = getResult();

        if (result == null || result.getBotpose() == null) {
            return null;
        }

        return new Pose2D(
                DistanceUnit.METER,
                result.getBotpose().getPosition().x,
                result.getBotpose().getPosition().y,
                AngleUnit.DEGREES,
                result.getBotpose().getOrientation().getYaw()
        );
    }

    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

}
