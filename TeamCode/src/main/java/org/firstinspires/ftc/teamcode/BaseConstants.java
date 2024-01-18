package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public final class BaseConstants{


    public static class Params{
        public static double RIGHT_CLAW_OPEN = 0.5;
        public static double LEFT_CLAW_OPEN = 0.8;
        public static double RIGHT_CLAW_CLOSE = 0.85;
        public  static double LEFT_CLAW_CLOSE = 0.5;
        public  static int HOME = -1088;
        public  static int LOW = 750;
        public  static int BACKDROP = 657;
        public  static int ARMHOME = 6;
        public  static double DEFAULT_SPEED = 0.4;
        public  static double WRIST_HOME_POSITION = 0.2;
        public  static double WRIST_SCORE_POSITION = 0.2;
        public  static double WRIST_PLACE_POSITION=0.87;


    }

}
