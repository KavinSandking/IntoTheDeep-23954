package org.firstinspires.ftc.teamcode.roadrunner.LastSeason.Localizer_Tuning;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;

public interface Localizer {
    Twist2dDual<Time> update();
}
