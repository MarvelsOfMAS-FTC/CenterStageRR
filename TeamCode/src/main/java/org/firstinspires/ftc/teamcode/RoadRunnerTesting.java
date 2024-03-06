package org.firstinspires.ftc.teamcode;

import android.view.animation.LinearInterpolator;

import androidx.core.graphics.PathSegment;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.QuinticSpline2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public class RoadRunnerTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-9, -72, 90));

            waitForStart();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(30, 30), Math.PI)
                            .splineTo(new Vector2d(60, 0), Math.PI)

                            .build());

        }
    }
}
