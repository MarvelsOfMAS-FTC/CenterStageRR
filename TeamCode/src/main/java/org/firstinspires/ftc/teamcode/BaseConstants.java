package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
@Config
public final class BaseConstants{

    public static class Params{
        public static class Camera{
            public static int LeftX = 150;
            public static int LeftY = 350;
            public static int Width = 60;
            public static int Height = 60;
            public static int MiddleX = 415;
            public static int MiddleY = 320;
            public static int RightX = 700;
            public static int RightY = 350;
        }

        //GENERIC MOTOR POWER
        public  double ZERO = 0.0;
        public  double FULL_PWR = 1.0;
        public  double FULL_PWR_INV = -1.0;

        //INTAKE MOTOR POWER
        public  double INTAKE_PWR = 0.85;

        //GENERIC HOME
        public  int HOME = 0;

        //CLIMB POS
        public  int CLIMB_HOME = 0; //INTAKE IN POS
        public  int CLIMB_INT_GND = 1150; //INTAKE OUT POS
        public  int CLIMB_INT_LVL_3 = 850; //INTAKE OUT POS
        public  int CLIMB_INT_LVL_5 = 800; //INTAKE OUT POS
        public  int CLIMB_LOW = 350;
        public  int CLIMB_MID = 450;
        public  int CLIMB_HIGH = 550;

        //SCORE POS
        public  double SCORE_HOME = 0.975;
        public  double SCORE_TRANSFER = 0.75;
        public  double SCORE_LOW = 0.44;
        public  double SCORE_MID = 0.46;
        public  double SCORE_HIGH = 0.46;

        //EXTEND POS
        public  double EXT_PWR = 0.75; //default power out
        public  double EXT_PWR_INV = -0.35; //default power out
        public  int EXT_HOME = -50;
        public  int EXT_RETRACT = 75;
        public  int EXT_PRELOAD = 300;
        public  int EXT_LOW = 325;
        public  int EXT_MID = 300;
        public  int EXT_HIGH = 375;

        //WRIST POS
        public   double WRIST_IN = 0.9;
        public   double WRIST_GND = 0.40;
        public   double WRIST_LVL_5 = 0.40;

        //FINGER POS
        public   double FINGER_IN = 0.9; //FINGER TUCKED IN
        public   double FINGER_GND = 0.395; //FINGER ON GND

        //INTAKE PWR


        //DRONE POS
        public   double DRONE_HOME = 0.8;
    }
    public static Params PARAMS = new Params();
    public static Params.Camera CAMERA = new Params.Camera();

}
