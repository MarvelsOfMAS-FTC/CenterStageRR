package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

@Autonomous
public class TestBuildRR extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;
    BaseRobotMethods robot;// Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;
    private VisionProcessor visionProcessor;// Used for managing the AprilTag detection process.
    double[] error;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BaseRobotMethods(hardwareMap);
        visionProcessor = robot.getVisionProcessor();
        visionPortal = robot.getPortal();
        aprilTag = robot.getAprilTag();
        robot.telemetry = this.telemetry;
        robot.parent = this;
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
        robot.useApriltag();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("CLIMBL",robot.climbl.getCurrentPosition());
            if(gamepad1.touchpad){
                summonAprilTag(1,drive);
            } else if (gamepad1.a) {
                Actions.runBlocking(new SequentialAction(
                        robot.intakeLevel5()
                ));
            }
            telemetry.update();
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*2,
                            -gamepad1.left_stick_x*2
                    ),
                    -gamepad1.right_stick_x
            ));
        }

    }

    public double[] aprilTagTrack(int DESIRED_TAG_ID){
        double[] arr = {0,0,0};
        boolean targetFound = false;
        AprilTagDetection desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {

            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    arr[0] = desiredTag.ftcPose.x;
                    arr[1] = desiredTag.ftcPose.y;
                    arr[2] = desiredTag.ftcPose.yaw;
                    error = arr;
                    break;
                }
            }
        }
        return arr;
    }
    public void summonAprilTag(int id,MecanumDrive drive) {
        aprilTagTrack(id);
        if(error == null){
            return;
        }
        sleep(250);
        telemetry.addData("Array: ", Arrays.toString(error));
        telemetry.update();

        Actions.runBlocking(drive
                .actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x,drive.pose.position.y-error[0]), new Rotation2d(drive.pose.heading.real, drive.pose.heading.imag).toDouble()+Math.toRadians(error[2]))
                .build());
        sleep(500);
    }

}
