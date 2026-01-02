package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Limelight {

    private static Limelight3A limelight;

    private Limelight() {}

    public static void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public static LLResult getResult() {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        return result;
    }

    public static double getTx() {
        LLResult result = getResult();
        return result != null ? result.getTx() : 0.0;
    }

    public static double getTy() {
        LLResult result = getResult();
        return result != null ? result.getTy() : 0.0;
    }


    public static double getDistance(
            double tagHeight
    ) {
        LLResult result = getResult();
        if (result == null) return 0.0;

        double ty = result.getTy();
        double angleRad = Math.toRadians(0 + ty);

        return (tagHeight - 0.3556) / Math.tan(angleRad);
    }

    public static boolean isDetected() {
        LLResult result = getResult();
        return result != null && result.isValid() && result.getBotpose() != null;
    }


    public static boolean hasTarget() {
        return getResult() != null;
    }
}

