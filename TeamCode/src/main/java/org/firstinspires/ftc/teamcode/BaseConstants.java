package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class BaseConstants extends LinearOpMode {
    public double RIGHT_CLAW_OPEN = -0.7;
    public double LEFT_CLAW_OPEN = 1;
    public double RIGHT_CLAW_CLOSE = 0.5;
    public  double LEFT_CLAW_CLOSE = 0.2;
    public  int LOW = 2000;
    public  int LOW_HOME = 0;
    public  double LOW_SPEED = 0.8;
    public  double WRIST_TEST_POSITION = 0.4;
    public  double WRIST_SCORE_POSITION = 0;
    public  double WRIST_PLACE_POSITION=0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }
}
