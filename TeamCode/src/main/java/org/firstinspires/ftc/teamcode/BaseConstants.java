package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
@Config
public final class BaseConstants{
    public static class  Params{

        //GENERIC MOTOR POWER
        public  double ZERO = 0.0;
        public  double FULL_PWR = 1.0;
        public  double FULL_PWR_INV = -1.0;

        //GENERIC HOME
        public  int HOME = 0;

        //CLIMB POS
        public  int CLIMB_HOME = 0; //INTAKE IN POS
        public  int CLIMB_INT_GND = 1200; //INTAKE OUT POS
        public int CLIMB_INT_LVL_5 = 876; //INTAKE OUT POS
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
        public static final double WRIST_IN = 0.87;
        public static final double WRIST_GND = 0.41;
        public static final double WRIST_LVL_5 = 0.42;

        public static final double WRIST_TRANSFER_PWR = 0.65;

        //FINGER POS
        public static final double FINGER_IN = 0.9; //FINGER TUCKED IN
        public static final double FINGER_GND = 0.4; //FINGER ON GND

        //DRONE POS
        public static final double DRONE_HOME = 0.8;


        //TIMER FUNCTIONS
        public static int TIMEOUT = 7;


    }

}
