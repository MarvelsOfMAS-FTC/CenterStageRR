package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
@Config
public final class BaseConstants{

    public static class  Params {
        public static class Camera{
            public static int LeftX = 150;
            public static int LeftY = 350;
            public static int Width = 60;
            public static int Height = 60;
            public static int MiddleX = 415;
            public static int MiddleY = 350;
            public static int RightX = 700;
            public static int RightY = 350;
        }

        //GENERIC MOTOR POWER
        public  double ZERO = 0.0;
        public  double FULL_PWR = 1.0;
        public  double FULL_PWR_INV = -1.0;

        //GENERIC HOME
        public  int HOME = 0;

        //CLIMB POS
        public  int CLIMB_HOME = 0; //INTAKE IN POS
        public  int CLIMB_INT_GND = 1200; //INTAKE OUT POS
        public  int CLIMB_INT_LVL_5 = 1000; //INTAKE OUT POS
        public  int CLIMB_LOW = 400;
        public  int CLIMB_MID = 550;
        public  int CLIMB_HIGH = 1000;

        //SCORE POS
        public  double SCORE_HOME = 0.9;
        public  double SCORE_BACKDROP = 0.44;
        public  double SCORE_MID = 0.8;

        //EXTEND POS
        public  double EXT_PWR = 0.65; //default power out
        public  int EXT_HOME = 0;
        public  int EXT_RETRACT = 75;
        public  int EXT_PRELOAD = 300;
        public  int EXT_LOW = 450;
        public  int EXT_MID = 470;
        public  int EXT_HIGH = 500;

        //WRIST POS
        public   double WRIST_IN = 0.87;
        public   double WRIST_GND = 0.41;
        public   double WRIST_LVL_5 = 0.42;

        public   double WRIST_TRANSFER_PWR = 0.65;

        //FINGER POS
        public   double FINGER_IN = 0.9; //FINGER TUCKED IN
        public   double FINGER_GND = 0.4; //FINGER ON GND

        //DRONE POS
        public   double DRONE_HOME = 0.8;


        //TIMER FUNCTIONS
        public   int TIMEOUT = 7;


    }
    public static Params PARAMS = new Params();
    public static Params.Camera CAMERA = new Params.Camera();

}
