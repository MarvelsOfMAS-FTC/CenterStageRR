package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class BaseConstants extends LinearOpMode {
    public static final double RIGHT_CLAW_OPEN = -0.7;
    public static final double LEFT_CLAW_OPEN = 1;
    public static final double RIGHT_CLAW_CLOSE = 0.5;
    public static final double LEFT_CLAW_CLOSE = 0.2;
    public static final int LOW = 2000;
    public static final double LOW_SPEED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
}
