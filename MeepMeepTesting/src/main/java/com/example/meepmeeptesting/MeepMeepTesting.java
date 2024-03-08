package com.example.meepmeeptesting;


import static com.example.meepmeeptesting.AUTO.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.*;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

   enum Robot{
        MAS,
        M12,
       BOTH,
    }
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800, 9999);
        RoadRunnerBotEntity m12 = null;
        RoadRunnerBotEntity mas = null;
        Robot robot = Robot.MAS;
        if(robot == Robot.BOTH){
            mas = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark()) // mas robot is blue
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                    .build();
            m12 = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark()) // m12 robot is red
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                    .build();
            meepMeep.addEntity(mas);
            meepMeep.addEntity(m12);
        }else  if (robot==Robot.M12){
            m12 = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark()) // m12 robot is red
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                    .build();
            meepMeep.addEntity(m12);
        }else if(robot==Robot.MAS){
            mas = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark()) // mas robot is blue
                    .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 12)
                    .build();
            meepMeep.addEntity(mas);
        }
        mas.runAction(mas.getDrive().actionBuilder(new Pose2d(AUTO.startPosX, AUTO.startPosY, Math.toRadians(90)))
                .lineToYLinearHeading(AUTO.spikeMarkPosY + AUTO.spikeMarkOffsetY, AUTO.tagMid)
                .waitSeconds(1.75+0.5) // t≅2.25

                //TURN TO BACKBOARD
                .strafeToLinearHeading(new Vector2d(27, AUTO.tagScorePoxY + AUTO.tagScoreOffsetY-7), AUTO.tagScoreHeading) // 180
                .turnTo(Math.toRadians(180.0000000000001))

                //PUSH IN AND SCORE
                .lineToX(AUTO.tagScorePosX)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY-7), tagScoreHeading)
                .waitSeconds(0.01)

                //GOTO STACK AND WAIT IF NEEDED
                .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY-8), tagScoreHeading)
                .waitSeconds(0.01) //added to make approach more gentle
                .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY-8), tagScoreHeading)
                .waitSeconds(0.75)

                //RETURN TO BACKBOARD AND SCORE
                .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY-6), tagScoreHeading)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY-4), tagScoreHeading)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY-8), tagScoreHeading)
                .waitSeconds(0.01) //added to make approach more gentle
                .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY-8), tagScoreHeading)
                .waitSeconds(0.75)

                //RETURN TO BACKBOARD AND SCORE
                .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY-6), tagScoreHeading)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY-4), tagScoreHeading)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(parkPosX, parkPosY-6), tagScoreHeading)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .start();
    }

}
