package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
@Autonomous
public class IsaaqRR extends LinearOpMode {
    double startposx = 0;
    double startposy = 0;
    double startheading = Math.toRadians(90);
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));
        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToY(24)
                        //.splineTo(new Vector2d(startposx+24,startposy+24), startheading)
                        .build());

    }
}
