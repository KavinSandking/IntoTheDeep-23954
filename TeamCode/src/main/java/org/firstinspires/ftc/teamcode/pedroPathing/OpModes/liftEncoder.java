package org.firstinspires.ftc.teamcode.pedroPathing.OpModes;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Slides;


@Config
@TeleOp(name = "Lift Encoder")

class liftEncoder extends NextFTCOpMode {
    public liftEncoder() {
        super(Slides.INSTANCE);
    }

    @Override
    public void onInit() {}
    @Override
    public void onStartButtonPressed() {

    }
}
