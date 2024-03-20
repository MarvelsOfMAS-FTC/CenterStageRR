package org.firstinspires.ftc.teamcode;

import android.util.Size;

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


@Autonomous
public class AutoCloseRR extends LinearOpMode {
    //VARIABLES-------------------------------------------------------------------------------------
    String Side = "blue";
    String spike;
    public int sideMult = 1; //default blue = 1, red = -1
    int sideBlueY = 1;
    int sideRedY = -1;

    public boolean cycleStack = true;
    public boolean insideRoute = false;
    public boolean waitBool = false;
    public int waitOnAlliance = 0; //default variable for waiting
    public int waitDuration = 1; //how long to wait on alliance


    //START POS ------------------------------------------------------------------------------------
    double startPosX = 12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITIONAutoCloseRR
    double startPosY = (72 + 0) * sideMult; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(90*sideMult);

    //SPIKE MARK POS -------------------------------------------------------------------------------
    double spikeHeading; //heading to score spike mark
    double spikeMarkPosY = 57*sideMult; //how far forward to move for the very first auto movement
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double spikeMarkLeft = Math.toRadians(120*sideMult);
    double spikeMarkLeftY = 6;
    double spikeMarkMid = Math.toRadians(100*sideMult);
    double spikeMarkRight = Math.toRadians(70*sideMult);
    double spikeMarkRightY = 6;
    
    //BACKDROP POS ---------------------------------------------------------------------------------

    double tagScorePosX = 38; //backdrop scoring center point for AprilTag 
    double tagScoreOffsetX = 6; //how much to push into the backdrop for the backup move
    double tagScorePoxY = 42*sideMult; //mid placement preload tag score pos Y
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreLeft = 5; //left offset
    double tagScoreRight = -6.3; //right offset
    double tagScoreHeading = Math.toRadians(180); //heading to score on backdrop
    double tagScoreHeadingOffset = Math.toRadians(4); //extra adjust for complicated moves

    //CYCLING POS ----------------------------------------------------------------------------------
    double pixelStackPosX = -56; //where the actual pixel stack is in X
    double pixelBeforeY = -5; //justify to the left of pixel stack
    double pixelAfterY = 4; //move to the right to spread pixels out
    double pixelSweepOffsetX = 5; //how much to backup to spread out
    double pixelHeading = Math.toRadians(20); //how much to turn to sweep the stack
    double pixelStackPosY = 43.2*sideMult;
    double cycleScorePosY = 44*sideMult; //used to dodge right pixel on transit

    double routeOffsetY = -26.5*sideMult; //how far from center tag to move for outside cycle run. Also sets default value.
    double routeOut = -26.5; //outside pixel stack
    double routeIn = 0; //inside pixel stack (in line with middle of backdrop)

    double routeWait = 0.5; //need more time for outside route. Also sets default value.
    double routeWaitIn = 0;
    double routeWaitOut = 0.5;

    //PARK POS -------------------------------------------------------------------------------------
    double parkPosX = 46; //default is to park near the corner of the field next to backdrop
    double parkPosY = 68*sideMult;
    double parkInY = 68;
    double parkOutY = 20;
    boolean OppositePark = false;

    //APRIL TAG VARAIABLES -------------------------------------------------------------------------
    AprilTagProcessor aprilTag;
    double[] error;
    VisionPortal visionPortal;
    VisionProcessor visionProcessor;
    double aprilTagOffsetX = 22; //how much to push into backdrop from zero for april tag
    double aprilTagOffsetY = 3;
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
                routeOffsetY = routeIn*sideMult;
                routeWait = routeWaitIn;
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

                if(Side.equalsIgnoreCase("Red")){
                    id=redM;
                }else {
                    id=blueM;
                }

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                spike="left";
                spikeHeading = spikeMarkLeft;
                tagScoreOffsetY = tagScoreLeft*sideMult;

                spikeMarkOffsetY = spikeMarkLeftY*sideMult;
                if(Side.equalsIgnoreCase("Red")){
                    id=redL;
                }else {
                    id=blueL;
                }

            } else {
                spike = "right";
                spikeHeading = spikeMarkRight;
                tagScoreOffsetY = tagScoreRight*sideMult;
                spikeMarkOffsetY = spikeMarkRightY*sideMult;

                if(Side.equalsIgnoreCase("Red")){
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
                    .strafeToLinearHeading(new Vector2d(tagScorePosX, tagScorePoxY + tagScoreOffsetY), tagScoreHeading + tagScoreHeadingOffset, drive.fastVelConstant, drive.fastAccelConstraint)
                    .waitSeconds(0.01)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            summonAprilTag(id,drive,robot);
            drive.updatePoseEstimate();

            if(waitBool){
                sleep(waitOnAlliance * 1000); //WAIT IF NEEDED FOR ALLIANCE
            }

            //CYCLE PIXEL STACK
            if (cycleStack) {
                Action pixelCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS ------------------------------------------------------------------
                        //RETRACT
                        .afterTime(0.5, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.3 + routeWait, robot.intakeLevel5())
                        .afterTime(5 + routeWait,robot.scoreMid())

                        //TRANSFER & SCORE
                        .afterTime(5.5 + routeWait, robot.transferA())
                        .afterTime(6 + routeWait, robot.transferB())
                        .afterTime(6.2 + routeWait, robot.transferC())
                        .afterTime(7 + routeWait, robot.intakeStop())

                        //MOVEMENT -----------------------------------------------------------------
                        //CENTER ROBOT ON STACK
                        .strafeToLinearHeading(new Vector2d(tagScorePosX, (cycleScorePosY + routeOffsetY)*sideMult), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                        .waitSeconds(0.01)

                        //GOTO STACK ON LEFT SIDE
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, (cycleScorePosY + routeOffsetY + pixelBeforeY)*sideMult), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                        .waitSeconds(0.1) //added to make approach more gentle

                        //BACKUP AND TURN AT THE SAME TIME TO SWEEP THE STACK
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelSweepOffsetX, (pixelStackPosY + routeOffsetY + pixelAfterY)*sideMult), (tagScoreHeading - pixelHeading*sideMult), drive.defaultVelConstraint)
                        .turnTo(tagScoreHeading) //turn back to pickup more pixels
                        .waitSeconds(1.25)

                        //RETURN TO BACKBOARD
                        .strafeToLinearHeading(new Vector2d(tagScorePosX, (cycleScorePosY + routeOffsetY + pixelBeforeY)*sideMult), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                        .waitSeconds(0.01)
                        //GO TO APRIL TAG SCORE POS
                        .strafeToLinearHeading(new Vector2d(tagScorePosX, tagScorePoxY*sideMult), tagScoreHeading, drive.defaultVelConstraint)
                        .endTrajectory()
                        .build();

                Actions.runBlocking(pixelCycle1);
                drive.updatePoseEstimate();

                //This controls where to drop off pixels on backdrop
                if(spike.equalsIgnoreCase("left")){
                    if(Side.equalsIgnoreCase("blue")){
                        summonAprilTag(blueM,drive,robot);
                    }else{
                        summonAprilTag(redM,drive,robot);
                    }

                }else if (spike.equalsIgnoreCase("middle")) {
                    if(Side.equalsIgnoreCase("blue")){
                        summonAprilTag(blueL,drive,robot);
                    }else{
                        summonAprilTag(redL,drive,robot);
                    }

                }else {
                    if(Side.equalsIgnoreCase("blue")){
                        summonAprilTag(blueM,drive,robot);
                    }else{
                        summonAprilTag(redM,drive,robot);
                    }
                }
            }

            //PARK THE ROBOT
            Action parkBot = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    .afterTime(0.5, robot.home())

                    //MOVEMENT ---------------------------------------------------------------------
                    .strafeToLinearHeading(new Vector2d(parkPosX, parkPosY), tagScoreHeading)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(parkBot);
            drive.updatePoseEstimate();
            break;
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
    public void summonAprilTag(int id,MecanumDrive drive,BaseRobotMethods robot) {
        aprilTagTrack(id);
        if(error == null){
            telemetry.addData("BACKDROP APRIL TAG NOT FOUND","");
            robot.sendTelemetry("BACKDROP APRIL TAG NOT FOUND","");
            telemetry.update();
            Actions.runBlocking(drive
                    .actionBuilder(drive.pose)
                    .afterTime(0.5, robot.low())
                    .afterTime(1.5, robot.mid())
                    .afterTime(2, robot.retract())
                    .lineToX(tagScorePosX+tagScoreOffsetX)
                    .build());
            return;
        }
        telemetry.addData("Array: ", Arrays.toString(error));
        robot.sendTelemetry("Array: ", Arrays.toString(error));
        telemetry.update();
        Actions.runBlocking(drive
                .actionBuilder(drive.pose)
                .afterTime(0.5, robot.low())
                .afterTime(1.5, robot.mid())
                .afterTime(2, robot.retract())
                .strafeToLinearHeading(new Vector2d(drive.pose.position.x-error[1]+aprilTagOffsetX,drive.pose.position.y-error[0] +aprilTagOffsetY*sideMult), new Rotation2d(drive.pose.heading.real, drive.pose.heading.imag).toDouble(), drive.fastVelConstant, drive.fastAccelConstraint)
                .waitSeconds(0.01)
                .build());
    }



}