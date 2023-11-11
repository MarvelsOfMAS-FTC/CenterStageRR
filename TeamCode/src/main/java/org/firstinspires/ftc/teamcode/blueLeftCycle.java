package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobotMethods;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;

public class blueLeftCycle extends LinearOpMode {
    public static double stack1x = 1;
    public static double stack1y = 1;
    public static double stack2x = 1;
    public static double stack2y = 1;
    public static double stack3x = 1;
    public static double stack3y = 1;

    public static double backboardX = 1;
    public static double backboardY = 1;

    public static int numStacks = 1; //cone stack params

    @Override
    public void runOpMode() throws InterruptedException {
        //HARDWARE INITIALIZATION ------------------------------------------------------------------
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
       // robot.telemetry = this.telemetry;
        //robot.parent = this;

        //SERVO INIT
        robot.initPos();
        robot.home();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(60, 0), Math.PI)
                        .build());
        );


    }


}
