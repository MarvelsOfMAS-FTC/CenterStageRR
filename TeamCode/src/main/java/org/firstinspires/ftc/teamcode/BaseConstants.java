package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public final class BaseConstants{


    public static class Params{
        public static double RIGHT_CLAW_OPEN = 0.5;
        public static double LEFT_CLAW_OPEN = 0;
        public static double RIGHT_CLAW_CLOSE = 1;
        public  static double LEFT_CLAW_CLOSE = 1;
        public  static int LOW = 1300;
        public  static int LOW_HOME = -1088;
        public  static double LOW_SPEED = 0.8;
        public  static double WRIST_TEST_POSITION = 0.4;
        public  static double WRIST_SCORE_POSITION = 0;
        public  static double WRIST_PLACE_POSITION=0;
    }

}
