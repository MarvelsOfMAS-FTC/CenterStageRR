package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

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

@Config
@Autonomous
public final class AutoCloseRR extends LinearOpMode {
    public static class Params{}
    public static Params PARAMS = new Params();
    //VARIABLES-------------------------------------------------------------------------------------
    String Side = "blue";
    String spike;
    public int sideMult = 1; //default blue = 1, red = -1
    int sideBlueY = 1;
    int sideRedY = -1;

    public boolean cycleStack = true; //to init the robot
    public boolean stackCycling = false; //to let april tag know if we're currently cycling
    public int cycles = 1; //how many cycles to run
    public boolean insideRoute = false;
    public boolean waitBool = false;
    public int waitOnAlliance = 0; //default variable for waiting
    public int waitDuration = 1; //how long to wait on alliance


    //START POS ------------------------------------------------------------------------------------
    public double startPosX = 12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITIONAutoCloseRR
    public double startPosY = (72 + 0); //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    public double startHeading = Math.toRadians(90);

    //SPIKE MARK POS -------------------------------------------------------------------------------
    public double spikeHeading; //heading to score spike mark
    public double spikeMarkPosY = 57.5; //how far forward to move for the very first auto movement
    public double spikeMarkOffsetY; //change spike mark tape forward movement
    public double spikeMarkLeft = Math.toRadians(120);
    public double spikeMarkLeftY = 6;
    public double spikeMarkMid = Math.toRadians(100);
    public double spikeMarkRight = Math.toRadians(70);
    public double spikeMarkRightY = 6;
    
    //BACKDROP POS ---------------------------------------------------------------------------------

    public double tagScorePosX = 38; //backdrop scoring center point for AprilTag
    public double tagScoreOffsetX = 12; //how much to push into the backdrop
    public double tagScoreCycleX = 2.5; //adjustment for tag position during cycle
    public double tagScorePoxY = 40; //mid placement preload tag score pos Y
    public double tagScoreOffsetY; //controls left-right preload displacement
    public double tagScoreLeft = 5; //left offset backdrop tag
    public double tagScoreRight = -6.25; //right offset backdrop tag
    public double tagScoreHeading = Math.toRadians(180); //heading to score on backdrop
    public double tagScoreHeadingOffset = Math.toRadians(4); //extra adjust for complicated moves

    //PIXEL STACK POS ----------------------------------------------------------------------------------
    public double pixelStackPosX = -57; //where the actual outside pixel stack is in X
    public double pixelStackPosY = 15; //where the actual outside pixel stack is in Y
    public double pixelSweepOffsetX = 4; //how much to backup to spread out
    
    public double pixelBeforeY = -4; //justify to the left of pixel stack
    public double pixelAfterY = 1; //move to the right to spread pixels out
    public double pixelHeading = Math.toRadians(10); //how much to turn to sweep the stack
    public double routeOffsetY = 0; //how far from outside stack to move for cycle run. Also sets default value.
    public double routeOut = 0; //outside pixel stack
    public double routeIn = 26; //offset by roughly a tile for inside stack

    public double routeWait = 0.5; //need more time for outside route. Also sets default value.
    public double routeWaitIn = 0;
    public double routeWaitOut = 0.5;

    //PARK POS -------------------------------------------------------------------------------------
    public double parkPosX = 50; //default is to park near the corner of the field next to backdrop
    public double parkPosY = 68; //sets default park pos
    public double parkInY = 68; //inside park pos
    public double parkOutY = 20; //outside park pos
    public boolean OppositePark = false;

    //APRIL TAG VARAIABLES -------------------------------------------------------------------------
    AprilTagProcessor aprilTag;
    double[] error;
    VisionPortal visionPortal;
    VisionProcessor visionProcessor;
    public double aprilTagOffsetY = 0;

    int id = 0;
    int blueL = 1; int blueM = 2; int blueR = 3;
    int redL = 4; int redM = 5; int redR = 6;

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        visionProcessor = robot.getVisionProcessor();
        visionPortal = robot.getPortal();
        aprilTag = robot.getAprilTag();
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //WAIT FOR START CODE ----------------------------------------------------------------------
        robot.home(); //initialize robot
        while (!opModeIsActive() && !isStopRequested())
        {
            //CYCLE PIXELS SELECT
            if(gamepad1.x) {
                cycleStack = true;
            } else if(gamepad1.b) {
                cycleStack = false;
            }

            //INSIDE CYCLE OR OUTSIDE?
            if(gamepad1.a) {
                insideRoute = true;
                routeOffsetY = routeOut*sideMult; //FAT FINGER SECURITY
                routeWait = routeWaitOut; //ONLY OUTSIDE CHANGE LATER
            } else if(gamepad1.y) {
                insideRoute = false;
                routeOffsetY = routeOut*sideMult; //one tile over to go for outside pixel stack
                routeWait = routeWaitOut;

                waitBool = false; //no time to wait on outside route
                waitOnAlliance = waitDuration;
            }

            //WAIT ON ALLIANCE?
            if(gamepad1.left_bumper) {
                waitBool = true;
                waitOnAlliance = waitDuration;
            } else if(gamepad1.right_bumper) {
                waitBool = false;
                waitOnAlliance = 0;
            }
            if(gamepad1.touchpad){
                OppositePark=!OppositePark;
            }
            if(gamepad1.right_trigger>0.1){
                Side = "blue";
            }else if (gamepad1.left_trigger>0.1){
                Side = "red";
            }
            if(Side.equalsIgnoreCase("blue")){
                sideMult = sideBlueY;
            }else{
                sideMult = sideRedY; //flip y for red side
            }
            if(OppositePark){
                parkPosY=parkOutY*sideMult; //inside park vs outside for y
            }else{
                parkPosY=parkInY*sideMult;
            }

            telemetry.addData("-- CLOSE AUTO --","");
            telemetry.addData("","");

            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.addData("Side : ",Side);
            telemetry.addData("","");

            telemetry.addData("Inside Route?: ", insideRoute);
            telemetry.addData("Cycle Stack?: ", cycleStack);
            telemetry.addData("Opposite Park : ",OppositePark);
            telemetry.addData("Wait on Partner?: ", waitBool);

            telemetry.addData("","");
            telemetry.addData("Press RIGHT TRIGGER for BLUE, LEFT TRIGGER for RED ","");
            telemetry.addData("Press A for INSIDE ROUTE, Y for OUT","");
            telemetry.addData("Press X to CYCLE, B to NOT CYCLE","");
            telemetry.addData("Press TOUCHPAD for OPPOSITE PARK","");
            telemetry.addData("Press LB to WAIT, RB to ZOOM ZOOM","");

            // dashboard telemetry
            robot.sendTelemetry("-- CLOSE AUTO --","");
            robot.newLine();
            robot.sendTelemetry("Cam Place: ",robot.visionProcessor.getSelection());
            robot.sendTelemetry("Side : ",Side);
            robot.newLine();
            robot.sendTelemetry("Inside Route?: ",insideRoute);
            robot.sendTelemetry("Cycle Stack?: ", cycleStack);
            robot.sendTelemetry("Opposite Park : ",OppositePark);
            robot.sendTelemetry("Wait on Partner?: ", waitBool);
            robot.newLine();
            robot.sendTelemetry("Press RIGHT TRIGGER for BLUE, LEFT TRIGGER for RED ","");
            robot.sendTelemetry("Press A for INSIDE ROUTE, Y for OUT","");
            robot.sendTelemetry("Press X to CYCLE, B to NOT CYCLE","");
            robot.sendTelemetry("Press TOUCHPAD for OPPOSITE PARK","");
            robot.sendTelemetry("Press LB to WAIT, RB to ZOOM ZOOM","");
            telemetry.update();
        }

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                spike="middle";
                spikeHeading = spikeMarkMid;
                tagScoreOffsetY = 0; //no score offset for middle tag
                spikeMarkOffsetY = 0; //how far forward to go to place pixel. should be zero.

                if(Side.equalsIgnoreCase("red")){
                    id=redM;
                }else {
                    id=blueM;
                }

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                spike="left";
                spikeHeading = spikeMarkLeft;
                tagScoreOffsetY = tagScoreLeft*sideMult;

                spikeMarkOffsetY = spikeMarkLeftY*sideMult;
                if(Side.equalsIgnoreCase("red")){
                    id=redL;
                }else {
                    id=blueL;
                }

            } else {
                spike = "right";
                spikeHeading = spikeMarkRight;
                tagScoreOffsetY = tagScoreRight*sideMult;
                spikeMarkOffsetY = spikeMarkRightY*sideMult;

                if(Side.equalsIgnoreCase("red")){
                    id=redR;
                }else {
                    id=blueR;
                }
            }
            //APRIL TAG INIT
            robot.useApriltag();

            //SCORE PRELOAD PIXELS
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPosX, startPosY, startHeading));
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1, robot.spikeScore())
                    .afterTime(1.5, robot.fingerHome())
                    .afterTime(2, robot.home())

                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(spikeMarkPosY + spikeMarkOffsetY, spikeHeading)
                    .waitSeconds(1.25)

                    //TURN TO BACKBOARD
                    .strafeToLinearHeading(new Vector2d(tagScorePosX, tagScorePoxY + tagScoreOffsetY), tagScoreHeading + tagScoreHeadingOffset, drive.defaultVelConstraint, drive.fastAccelConstraint)
                    .waitSeconds(0.25)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            aprilTagAlignment(id,drive,robot);
            drive.updatePoseEstimate();

            if(waitBool){
                sleep(waitOnAlliance * 1000); //WAIT IF NEEDED FOR ALLIANCE
            }

            //CYCLE PIXEL STACK
            if (cycleStack) {
                stackCycling = true; //for april tag alignment
                for(int i=0; i<cycles; i++){
                    Action pixelCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                            //ACTIONS ------------------------------------------------------------------
                            //RETRACT
                            .afterTime(0, robot.home())

                            //WHIP OUT INTAKE & FEED
                            .afterTime(2.15 + routeWait, robot.intakeGround())
                            .afterTime(3 + routeWait, robot.intakeLevel3())

                            //TRANSFER & SCORE
                            .afterTime(6 + routeWait, robot.transferA())
                            .afterTime(6.5 + routeWait, robot.transferB())
                            .afterTime(7 + routeWait, robot.transferC())
                            .afterTime(8 + routeWait, robot.intakeStop())

                            //MOVEMENT -----------------------------------------------------------------
                            //CENTER ROBOT ON STACK
                            .strafeToLinearHeading(new Vector2d(tagScorePosX, (pixelStackPosY + routeOffsetY) * sideMult), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                            .waitSeconds(0.01)

                            //GOTO STACK ON LEFT SIDE
                            .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelSweepOffsetX, (pixelStackPosY + routeOffsetY + pixelBeforeY) * sideMult), tagScoreHeading, drive.defaultVelConstraint, drive.fastAccelConstraint)
                            .waitSeconds(0.01) //added to make approach more gentle

                            //BACKUP AND TURN AT THE SAME TIME TO SWEEP THE STACK
                            .strafeToLinearHeading(new Vector2d(pixelStackPosX, (pixelStackPosY + routeOffsetY + pixelAfterY) * sideMult), (tagScoreHeading - pixelHeading * sideMult), drive.slowVelConstant, drive.slowAccelConstraint)
                            .waitSeconds(0.5)
                            .turnTo(tagScoreHeading)
                            .waitSeconds(1)

                            //RETURN TO BACKBOARD
                            .strafeToLinearHeading(new Vector2d(tagScorePosX + tagScoreCycleX, (pixelStackPosY + routeOffsetY) * sideMult), tagScoreHeading, drive.defaultVelConstraint, drive.fastAccelConstraint)
                            .waitSeconds(0.01)
                            //GO TO APRIL TAG SCORE POS
                            .strafeToLinearHeading(new Vector2d(tagScorePosX + tagScoreCycleX, tagScorePoxY * sideMult), tagScoreHeading, drive.defaultVelConstraint)
                            .endTrajectory()
                            .build();

                    Actions.runBlocking(pixelCycle1);
                    drive.updatePoseEstimate();

                    //This controls where to drop off pixels on backdrop
                    if (spike.equalsIgnoreCase("left")) {
                        if (Side.equalsIgnoreCase("blue")) {
                            aprilTagAlignment(blueM, drive, robot);
                        } else {
                            aprilTagAlignment(redM, drive, robot);
                        }

                    } else if (spike.equalsIgnoreCase("middle")) {
                        if (Side.equalsIgnoreCase("blue")) {
                            aprilTagAlignment(blueL, drive, robot);
                        } else {
                            aprilTagAlignment(redR, drive, robot);
                        }

                    } else {
                        if (Side.equalsIgnoreCase("blue")) {
                            aprilTagAlignment(blueM, drive, robot);
                        } else {
                            aprilTagAlignment(redM, drive, robot);
                        }
                    }
                }
            }

            //PARK THE ROBOT
            Action parkBot = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    .afterTime(0, robot.home())

                    //MOVEMENT ---------------------------------------------------------------------
                    .waitSeconds(0.25)
                    .strafeToLinearHeading(new Vector2d(parkPosX, parkPosY), tagScoreHeading)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(parkBot);
            drive.updatePoseEstimate();
            break;
        }
    }
    public double[] aprilTagTrack(int DESIRED_TAG_ID, BaseRobotMethods robot){
        telemetry.addData("ID VALUE",DESIRED_TAG_ID);
        robot.sendTelemetry("ID VALUE",DESIRED_TAG_ID);
        telemetry.update();
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
                    telemetry.addData("error",error);
                    robot.sendTelemetry("error",error);
                    telemetry.update();
                    break;
                }
            }
        }
        return arr;
    }
    public void aprilTagAlignment(int id,MecanumDrive drive, BaseRobotMethods robot) {
        sleep (250); //for everything to settle
        aprilTagTrack(id, robot);
        //error = null;
        if(error == null){
            telemetry.addData("BACKDROP APRIL TAG NOT FOUND","");
            robot.sendTelemetry("BACKDROP APRIL TAG NOT FOUND","");
            telemetry.update();
            if(stackCycling == false) { //if preload run this (april tag failed)
                Actions.runBlocking(drive
                        .actionBuilder(drive.pose)
                        .afterTime(0, robot.low())
                        .afterTime(2, robot.high())
                        .afterTime(2.5, robot.retract())
                        .waitSeconds(0.5)
                        .turnTo(tagScoreHeading)
                        .lineToX(drive.pose.position.x + tagScoreOffsetX)
                        .waitSeconds(1)
                        .build());
                return;
            }
            else{ //if cycling run this
                Actions.runBlocking(drive
                        .actionBuilder(drive.pose)
                        .afterTime(0, robot.mid())
                        .afterTime(2, robot.high())
                        .afterTime(2.5, robot.retract())
                        .waitSeconds(0.5)
                        .turnTo(tagScoreHeading)
                        .lineToX(drive.pose.position.x + tagScoreOffsetX)
                        .waitSeconds(1)
                        .build());
                return;
            }
        }
        telemetry.addData("Array: ", Arrays.toString(error));
        robot.sendTelemetry("Array: ", Arrays.toString(error));
        telemetry.update();
        if(stackCycling == false) { //if preload run this (april tag succeeded)
            Actions.runBlocking(drive
                    .actionBuilder(drive.pose)
                    .afterTime(0, robot.low())
                    .afterTime(2, robot.high())
                    .afterTime(2.5, robot.retract())
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - error[0] + aprilTagOffsetY * sideMult), drive.pose.heading, drive.fastVelConstant, drive.fastAccelConstraint)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x + tagScoreOffsetX, drive.pose.position.y - error[0] + aprilTagOffsetY * sideMult), drive.pose.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                    .waitSeconds(1)
                    .build());
        }
        else{ //if cycling run this (april tag succeeded)
            Actions.runBlocking(drive
                    .actionBuilder(drive.pose)
                    .afterTime(0, robot.mid())
                    .afterTime(2, robot.high())
                    .afterTime(2.5, robot.retract())
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - error[0] + aprilTagOffsetY * sideMult), drive.pose.heading, drive.fastVelConstant, drive.fastAccelConstraint)
                    .strafeToLinearHeading(new Vector2d(drive.pose.position.x + tagScoreOffsetX, drive.pose.position.y - error[0] + aprilTagOffsetY * sideMult), drive.pose.heading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                    .waitSeconds(1)
                    .build());
        }
    }
}