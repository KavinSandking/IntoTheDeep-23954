package org.firstinspires.ftc.teamcode.roadrunner.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public class CloseClaw implements Action {
        private boolean initialized = false;
        private long startTime;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                claw.setPosition(0.0);
                startTime = System.currentTimeMillis();
                initialized = true;
            }

            long elapsedTime = System.currentTimeMillis() - startTime;

            if (elapsedTime >= 350) {
                return false;
            }
            return true;
        }
    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        private long startTime;
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                claw.setPosition(1.0);
                initialized = true;
            }
            return false;
        }
    }

    public Action openClaw() {
        return new OpenClaw();
    }
}