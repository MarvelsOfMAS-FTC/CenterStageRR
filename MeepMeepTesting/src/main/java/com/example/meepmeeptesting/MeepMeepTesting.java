package com.example.meepmeeptesting;


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
        MeepMeep meepMeep = new MeepMeep(800, 999);
        RoadRunnerBotEntity m12 = null;
        RoadRunnerBotEntity mas = null;
        Robot robot = Robot.MAS;
        if(robot == Robot.BOTH){
            mas = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark()) // mas robot is blue
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();
            m12 = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark()) // m12 robot is red
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();
            meepMeep.addEntity(mas);
            meepMeep.addEntity(m12);
        }else  if (robot==Robot.M12){
            m12 = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeRedDark()) // m12 robot is red
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();
            meepMeep.addEntity(m12);
        }else if(robot==Robot.MAS){
            mas = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setColorScheme(new ColorSchemeBlueDark()) // mas robot is blue
                    .setConstraints(120, 120, Math.toRadians(360), Math.toRadians(360), 15)
                    .build();
            meepMeep.addEntity(mas);
        }
        mas.runAction(mas.getDrive().actionBuilder(new Pose2d(AUTO.startPosX, AUTO.startPosY, AUTO.tagMid))
                .lineToYLinearHeading(AUTO.spikeMarkPosY + AUTO.spikeMarkOffsetY, AUTO.tagMid)
                .waitSeconds(1.75+0.5) // t≅2.25

                //TURN TO BACKBOARD
                .strafeToLinearHeading(new Vector2d(27, AUTO.tagScorePoxY + AUTO.tagScoreOffsetY), AUTO.tagScoreHeading) // 180
                .turnTo(Math.toRadians(180.0000000000001))

                //PUSH IN AND SCORE
                .lineToX(AUTO.tagScorePosX)
                .waitSeconds(0.01)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .start();
    }
    public static void runAction(RoadRunnerBotEntity robot){

    }

}
