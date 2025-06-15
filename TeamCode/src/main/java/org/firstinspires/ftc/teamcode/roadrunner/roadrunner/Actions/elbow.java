package org.firstinspires.ftc.teamcode.roadrunner.roadrunner.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class elbow {
    private Servo elbow;
    private Servo elbow2;

    public elbow(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(Servo.class, "elbow");
        elbow2 = hardwareMap.get(Servo.class, "elbow_2");
    }
    public class elbowStraight implements Action {
        private boolean initialized = false;
        private long startTime;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                elbow.setPosition(0.5);
                elbow2.setPosition(0.45);
                startTime = System.currentTimeMillis();
                initialized = true;
            }

            long elapsedTime = System.currentTimeMillis() - startTime;

            if (elapsedTime >= 150){
                return false;
            }
            return true;
        }
    }

    public Action elbowStraight() {
        return new elbowStraight();
    }
    public class lowerElbow implements Action {
        private boolean initialized = false;
        private long startTime;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                elbow.setPosition(0.188);
                elbow2.setPosition(0.878);
                startTime = System.currentTimeMillis();
                initialized = true;
            }

            long elapsedTime = System.currentTimeMillis() - startTime;

            if (elapsedTime >= 550) {
                return false;
            }
            return true;
        }
    }

    public Action lowerElbow() {
        return new elbow.lowerElbow();
    }

    public class elbowUp implements Action {
        private boolean initialized = false;
        private long startTime;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                elbow.setPosition(0.5);
                elbow2.setPosition(0.6);
                startTime = System.currentTimeMillis();
                initialized = true;
            }

            long elapsedTime = System.currentTimeMillis() - startTime;

            if (elapsedTime >=750 ) {
                return false;
            }
            return true;
        }
    }

    public Action elbowUp() {
        return new elbow.elbowUp();
    }
}