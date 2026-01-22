package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Tools.Constants.CAMERA_HEIGHT_INCHES;

public final class Limelight {

    private static Limelight3A limelight;

   // private static AdjustLL adjust;

    private Limelight() {}

    public static void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();


     //   adjust.init(hardwareMap);
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

    public static double getX() {
        LLResult result = getResult();
        if (result == null) {
            return 0;
        }
        return result.getBotpose().getPosition().x;
    }

    public static double getY() {
        LLResult result = getResult();
        if (result == null) {
            return 0;
        }
        return result.getBotpose().getPosition().y;
    }

    public static boolean isConnected() {
        return limelight.isConnected();
    }

    public static double getDistance(
            double tagHeight
    ) {
       // double cameraAngle = adjust.getDegrees();
        double cameraAngle = 0;
        double cameraHeightM = CAMERA_HEIGHT_INCHES * 39.37;

        LLResult result = getResult();
        if (result == null) return 0.0;

        double ty = result.getTy();
        double angleRad = Math.toRadians(cameraAngle + ty);

        return (tagHeight - cameraHeightM) / Math.tan(angleRad);
    }


    public static boolean isDetected() {
        LLResult result = getResult();
        return result != null && result.isValid() && result.getBotpose() != null;
    }


    // adjustable LL

//    public static void autoAdjustToTagPID() {
//        LLResult result = getResult();
//        if (result == null) return;
//
//        double ty = result.getTy();
//
//        double output = tiltPID.updatePID(ty);
//
//        output = Math.max(-MAX_STEP, Math.min(output, MAX_STEP));
//
//        double currentAngle = adjust.getDegrees();
//        double newAngle = currentAngle + output;
//
//        newAngle = Math.max(MIN_ANGLE, Math.min(newAngle, MAX_ANGLE));
//
//        adjust.setDegrees(newAngle);
//    }



    public static boolean hasTarget() {
        return getResult() != null;
    }
}

