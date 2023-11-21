package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonClose extends LinearOpMode {

    //VARIABLES---------------------------------------------------------------------------------------------------------------
    double startposx = 0;
    double startposy = 0;
    double startheading = Math.toRadians(90);
    @Override
    public void runOpMode() throws InterruptedException {

        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));
        //CAMERA INITIALIZATION --------------------------------------------------------------------

        //EXECUTE ACTIONS -----------------------------------------------------------------

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToY(24)
                        //.splineTo(new Vector2d(startposx+24,startposy+24), startheading)
                        .build());

    }
}
