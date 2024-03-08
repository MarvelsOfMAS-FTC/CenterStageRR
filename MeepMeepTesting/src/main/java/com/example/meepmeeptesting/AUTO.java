package com.example.meepmeeptesting;

public class AUTO {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    static public boolean cycleStack = true;

    static public boolean insideRoute = true;

    static public boolean waitBool = false;
    static public int waitDuration; //how long to wait on partner alliance in seconds

    //START POS
    static double startPosX = 12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    static double startPosY = 72 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    static double startHeading = Math.toRadians(180);

    //PRELOAD POS
    static double spikeMarkPosY = 55;
    static double spikeMarkOffsetY; //change spike mark tape forward movement
    static double tagHeading;
    static double tagLeft = Math.toRadians(120);
    static double tagMid = Math.toRadians(100);
    static double tagRight = Math.toRadians(72);

    static double tagScorePosX = 48; //center preload tag score pos X
    static double tagScorePoxY = 42; //center preload tag score pos Y
    static double tagScoreOffsetY; //controls left-right preload displacement
    static double tagScoreHeading = Math.toRadians(180);


    //CYCLING POS
    static double pixelStackPosX = -55; //how far into back wall to drive
    static double pixelStackOffsetX = -2.5;
    static double pixelStackPosY = 43.2;
    static double pixelStackOffsetY = -2.2;

    static double cycleScorePosX = 48; //push in more than tag score
    static double cycleScoreOffsetX = 1;
    static double cycleScorePosY = 44; //used to dodge right pixel on transit

    static double routeOffsetY; //how far from center tag to move for outside cycle run
    static double routeWait; //need more time for outside route

    //PARK POS
    static double parkPosX = 46;
    static double parkPosY = 68;
    static boolean OppositePark=false;
}