package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

public class Intake extends SDKSubsystem {
    public static final Intake INSTANCE = new Intake();

    public static Servo elbow, elbow2;
    public static Telemetry telemetry;
    public static Servo Claw;
    private double elbow1Target = 0.5; // Initialize target positions!
    private double elbow2Target = 0.45;
    private static double ELBOW_TOLERANCE = 0.01;
    private double clawCloseTarget = 1.0;
    private double clawOpenTarget = 0.0;
    private Intake() {}
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {
    }


    public void preInitUserHook(@NonNull Wrapper opMode) {
        HardwareMap hwMap = opMode.getOpMode().hardwareMap;
        telemetry = opMode.getOpMode().telemetry;
        elbow = hwMap.get(Servo.class, "elbow");
        elbow2 = hwMap.get(Servo.class, "elbow_2");
        Claw = hwMap.get(Servo.class, "claw");

        setDefaultCommand(maintainElbowPositions());
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

    public static void setPosition(double position, double position2){
        elbow.setPosition(position);
        elbow2.setPosition(position2);
    }
    public static void setClawPosition(double position){
        Claw.setPosition(position);
    }

    public double getElbowPosition(){
        return elbow.getPosition();
    }
    public double getElbow2Position(){
        return elbow2.getPosition();
    }
    public Lambda setPositionCommand(double position, double position2){
        return new Lambda("set - position")
                .setInterruptible(true)
                .setExecute(() -> {
                    elbow1Target = position;  // Update the target!
                    elbow2Target = position2;  // Update the target!
                    setPosition(position, position2);
                })
                .setFinish(() -> {
                    double diff1 = Math.abs(getElbowPosition() - position);
                    double diff2 = Math.abs(getElbow2Position() - position2);
                    return diff1 <= ELBOW_TOLERANCE && diff2 <= ELBOW_TOLERANCE;
                });
    }
    private Lambda maintainElbowPositions() {
        return new Lambda("maintain-elbows")
                .addRequirements(this)
                .setInterruptible(true)
                .setExecute(() -> {
                    setPosition(elbow1Target, elbow2Target); // Use the target variables!
                    telemetry.addData("Elbow1 Target", elbow1Target);
                    telemetry.addData("Elbow2 Target", elbow2Target);
                    telemetry.addData("Elbow1 Position", getElbowPosition());
                    telemetry.addData("Elbow2 Position", getElbow2Position());
                    telemetry.update();
                })
                .setFinish(() -> false); // Keep running!
    }
    public Lambda closeClaw(){
        return new Lambda("closeClaw")
                .addRequirements(this)
                .setInterruptible(true)
                .setExecute(() -> {
                    setClawPosition(clawCloseTarget);
                })
                .setFinish(() ->true);
    }

    public Lambda openClaw(){
        return new Lambda("openClaw")
                .addRequirements(this)
                .setInterruptible(true)
                .setExecute(() -> {
                    setClawPosition(clawOpenTarget);
        });
    }

}
