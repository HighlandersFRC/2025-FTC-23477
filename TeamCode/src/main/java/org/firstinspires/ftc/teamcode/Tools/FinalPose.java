package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;

public class FinalPose {
    public static double x;
    public static double y;
    public static double yaw;

    public static void setfinalPose(double X, double Y, double yaw) {
        x = X;
        y = Y;
        FinalPose.yaw = yaw;

    }

    public static void poseUpdate() {
        FieldOfMerit.processTags();
    }
}