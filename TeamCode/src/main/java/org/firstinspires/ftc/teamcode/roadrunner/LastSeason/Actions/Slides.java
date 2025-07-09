package org.firstinspires.ftc.teamcode.roadrunner.LastSeason.Actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


    public class Slides {
        private DcMotorEx Slides;
        private DcMotorEx Slides2;

        public Slides(HardwareMap hardwareMap) {
            Slides = hardwareMap.get(DcMotorEx.class, "Slides");
            Slides2 = hardwareMap.get(DcMotorEx.class, "Slides2");

            Slides.setDirection(DcMotorSimple.Direction.REVERSE);
            Slides2.setDirection(DcMotorSimple.Direction.REVERSE);
            Slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class slidesUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {

                    Slides.setTargetPosition(4050);
                    Slides2.setTargetPosition(4050);

                    Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Slides.setPower(1.0);
                    Slides2.setPower(1.0);

                    initialized = true;
                }

                packet.put("Slide Encoder 1", Slides.getCurrentPosition());
                packet.put("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.addData("Slide Encoder 1", Slides.getCurrentPosition());
                telemetry.addData("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.update();

                if (!Slides.isBusy() && !Slides2.isBusy()) {
                    Slides.setPower(0);
                    Slides2.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public Action slidesUp() {
            return new Slides.slidesUp();
        }

        public class slidesDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Slides.setTargetPosition(80);
                    Slides2.setTargetPosition(80);

                    Slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slides2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Slides.setPower(1.0);
                    Slides2.setPower(1.0);

                    initialized = true;
                }

                packet.put("Slide Encoder 1", Slides.getCurrentPosition());
                packet.put("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.addData("Slide Encoder 1", Slides.getCurrentPosition());
                telemetry.addData("Slide Encoder 2", Slides2.getCurrentPosition());
                telemetry.update();

                if (!Slides.isBusy() && !Slides2.isBusy()) {
                    Slides.setPower(0);
                    Slides2.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public class holdSlides implements Action {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Slides.setPower(0.1); // Small power to hold position
                    Slides2.setPower(0.1);
                    startTime = System.currentTimeMillis(); // Record start time
                    initialized = true;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                packet.put("Slide Encoder 1", Slides.getCurrentPosition());
                packet.put("Slide Encoder 2", Slides2.getCurrentPosition());
                packet.put("Hold Time (ms)", elapsedTime);

                // Run for 1 second (or adjust as needed for claw to open)
                return elapsedTime < 500;
            }
        }

        public Action holdSlides() {
            return new Slides.holdSlides();
        }


        public Action slidesDown() {
            return new Slides.slidesDown();
        }
    }

