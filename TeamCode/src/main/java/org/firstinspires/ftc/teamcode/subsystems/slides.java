package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;


@Config
public class slides extends SDKSubsystem {
    public static final slides INSTANCE = new slides();
    public static DcMotorEx Slides;
    public static DcMotorEx Slides2;
    public static final int HIGHBASKET = 4200;
    public static final int HOME = 100;
    public static double kp = 0.0007, ki = 0, kd = 0.0001;
    public static double kf = 0.001;
    public static Telemetry telemetry;
    public static PIDFController controller = new PIDFController(kp, ki, kd, kf);
    public static volatile boolean isPidActive = true;
    private int targetPosition = HIGHBASKET; // Start at HOME
    private boolean manualControl = false; // Flag for manual control

    private slides() {
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {
    }


    public void preInitUserHook(@NonNull Wrapper opMode) {
        HardwareMap hwMap = opMode.getOpMode().hardwareMap;
        telemetry = opMode.getOpMode().telemetry;
        Slides = hwMap.get(DcMotorEx.class, "Slides");
        Slides2 = hwMap.get(DcMotorEx.class, "Slides2");
        Slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slides.setDirection(DcMotorSimple.Direction.REVERSE);
        Slides2.setDirection(DcMotorSimple.Direction.REVERSE);

        setDefaultCommand(runPID()); // Only ONE default command!
    }

    @Override
    public void postUserLoopHook(@NonNull Wrapper opMode) {
    }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }
    public Lambda setManualPowerCommand(double power) {
        return new Lambda("manual-control")
                .addRequirements(this)
                .setInterruptible(true)
                .setExecute(() -> {
                    isPidActive = false; // Disable PID during manual control
                    setPower(power);
                })
                .setEnd((p) -> {
                    isPidActive = true;  // Re-enable PID when manual control stops
                    CurrentPosition();
                });
    }
    public Lambda runPID() {
        return new Lambda("pid")
                .addRequirements(this)
                .setInterruptible(true)
                .setExecute(() -> {
                    if (isPidActive && !manualControl) { // Check manualControl flag
                        double currentPosition = CurrentPosition();
                        double power = controller.calculate(currentPosition, targetPosition);
                        double ff = kf;
                        setPower(power + ff);

                        telemetry.addData("Slides Position", currentPosition);
                        telemetry.addData("Slides Target", targetPosition);
                        telemetry.addData("Slides Power", Slides.getPower());
                        telemetry.update();
                    }
                })
                .setFinish(() -> false);
    }

    public int CurrentPosition(){
        return (Slides.getCurrentPosition() + Slides2.getCurrentPosition())/2;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setPower(double power) { // Simplified - both motors get same power
        Slides.setPower(power);
        Slides2.setPower(power);
    }


    public void setTargetPosition(int target) {
        this.targetPosition = target; // Change the target!
    }



}