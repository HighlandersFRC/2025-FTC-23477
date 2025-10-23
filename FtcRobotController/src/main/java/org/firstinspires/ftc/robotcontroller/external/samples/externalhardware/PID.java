package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;
    public class PID {
        private final double kP;
        private final double kI;
        private final double kD;
        private double lastError = 0;
        private double integral = 0;

        public PID(double kp, double ki, double kd) {
            this.kP = kp;
            this.kI = ki;
            this.kD = kd;
        }

        public double update(double error) {
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            return (kP * error) + (kI * integral) + (kD * derivative);
        }

        public void reset() {
            lastError = 0;
            integral = 0;
        }
    }

