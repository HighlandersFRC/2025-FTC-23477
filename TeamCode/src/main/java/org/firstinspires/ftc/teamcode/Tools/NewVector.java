package org.firstinspires.ftc.teamcode.Tools;

public class NewVector {
    private double x;      // meters
    private double y;      // meters
    private double theta;  // radians

    public NewVector() {
        this.x = 0;
        this.y = 0;
        this.theta = 0;
    }

    public NewVector(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public NewVector(double magnitudeMeters, double thetaRadians, boolean polar) {
        this.x = magnitudeMeters * Math.cos(thetaRadians);
        this.y = magnitudeMeters * Math.sin(thetaRadians);
        this.theta = thetaRadians;
    }

    public void setCartesian(double xMeters, double yMeters) {
        this.x = xMeters;
        this.y = yMeters;
        this.theta = Math.atan2(y, x);
    }

    public void setPolar(double magnitudeMeters, double thetaRadians) {
        this.x = magnitudeMeters * Math.cos(thetaRadians);
        this.y = magnitudeMeters * Math.sin(thetaRadians);
        this.theta = thetaRadians;
    }

    public void reset() {
        x = 0;
        y = 0;
        theta = 0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public NewVector normalize() {
        double mag = magnitude();
        if (mag != 0) {
            return new NewVector(x / mag, y / mag, Math.atan2(y, x));
        }
        return new NewVector(0, 0, 0);
    }

    public NewVector scale(double s) {
        return new NewVector(x * s, y * s, theta);
    }

    public double dot(NewVector v) {
        return x * v.getX() + y * v.getY();
    }

    public NewVector rotate(double angleRadians) {
        double cosA = Math.cos(angleRadians);
        double sinA = Math.sin(angleRadians);
        double newX = x * cosA - y * sinA;
        double newY = x * sinA + y * cosA;
        return new NewVector(newX, newY, Math.atan2(newY, newX));
    }

    @Override
    public String toString() {
        return "NewVector(x=" + x + "m, y=" + y + "m, θ=" + theta + "rad)";
    }
}
