package org.firstinspires.ftc.teamcode.roadrunner.subsystems;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class CommandGroups {
    public static Command dropHighBasket() {
        return new Sequential(
                new Lambda("set-slides-high")
                        .addRequirements(slides.INSTANCE)
                        .setExecute(() -> slides.INSTANCE.setTargetPosition(slides.HIGHBASKET)), // Set to HIGHBASKET
                slides.INSTANCE.runPID(), // Assuming runPID is set up correctly
                Intake.INSTANCE.setPositionCommand(0.45, 0.645),
                new Wait(1)
        );
    }
    public static Command pickUpSample(){
        return new Sequential(
                new Lambda("set-slides-low")
                        .addRequirements(slides.INSTANCE)
                        .setExecute(() -> slides.INSTANCE.setTargetPosition(slides.HOME)), // Set to HIGHBASKET
                slides.INSTANCE.runPID(), // Assuming runPID is set up correctly
                Intake.INSTANCE.setPositionCommand(0.188, 0.878),
                new Wait(1)
        );
    }
}
