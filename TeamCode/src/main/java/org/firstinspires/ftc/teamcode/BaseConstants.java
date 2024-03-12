package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
@Config
public final class BaseConstants{
    public static class  Params{

        //GENERIC MOTOR POWER
        public static final double ZERO = 0.0;
        public static final double FULL_PWR = 1.0;
        public static final double FULL_PWR_INV = -1.0;

        //GENERIC HOME
        public static final int HOME = 0;

        //CLIMB POS
        public static final int CLIMB_HOME = 0; //INTAKE IN POS
        public static final int CLIMB_INT_GND = 1200; //INTAKE OUT POS
        public static final int CLIMB_INT_LVL_5 = 900; //INTAKE OUT POS
        public static final int CLIMB_INT_LVL_3 = 450; //INTAKE OUT POS
        public static final int CLIMB_LOW = 400;
        public static final int CLIMB_MID = 600;
        public static final int CLIMB_HIGH = 1000;

        //SCORE POS
        public static final double SCORE_HOME = 0.9;
        public static final double SCORE_BACKDROP = 0.52;

        //EXTEND POS
        public static final double EXT_PWR = 0.65; //default power out
        public static final int EXT_HOME = 0;
        public static final int EXT_RETRACT = 75;
        public static final int EXT_PRELOAD = 300;
        public static final int EXT_LOW = 520;
        public static final int EXT_MID = 650;
        public static final int EXT_HIGH = 760;

        //WRIST POS
        public static final double WRIST_IN = 0.87;
        public static final double WRIST_GND = 0.41;
        public static final double WRIST_LVL_5 = 0.4484;
        public static final double WRIST_LVL_3 = 0.395;
        public static final double WRIST_TRANSFER_PWR = 0.75;

        //FINGER POS
        public static final double FINGER_IN = 0.9; //FINGER TUCKED IN
        public static final double FINGER_GND = 0.35; //FINGER ON GND

        //DRONE POS
        public static final double DRONE_HOME = 0.8;


        //TIMER FUNCTIONS
        public static int TIMEOUT = 7;


    }

}
