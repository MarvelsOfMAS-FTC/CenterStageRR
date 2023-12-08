package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueFar extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public String fieldSide = "Blue";
    public boolean cycleStack = true;

    public boolean outsideRoute = true;

    public boolean waitBool = false;
    public int waitDuration; //how long to wait on partner alliance in seconds

    //START POS
    double startPosX = -36 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startPosY = 72 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(90);

    //PRELOAD POS
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double tagHeading;
    double tagLeft = Math.toRadians(119);
    double tagMid = Math.toRadians(100);
    double tagRight = Math.toRadians(72);

    double tagScorePosX = 45; //center preload tag score pos X
    double tagScorePoxY = 38; //center preload tag score pos Y -- THIS NEEDS TO BE DEAD CENTER FOR MIDDLE PLACEMENT ON BACK DROP --
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreHeading = Math.toRadians(180);


    //CYCLING POS
    double pixelStackPosX = -55; //how far into back wall to drive
    double pixelStackOffsetX = -2.5;
    double pixelStackPosY = 41;
    double cycleScorePosX = 45; //push in more than tag score
    double cycleScoreOffsetX = 1;
    double cycleScorePosY = 44; //used to dodge right pixel on transit

    double routeOffsetY  = -22; //how far from center tag to move for outside cycle run
    double routeWait = 0.5; //need more time for outside route

    //PARK POS
    double parkPosX = 55;
    double parkPosY = 18;

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            //CYCLE PIXELS SELECT
            if(gamepad1.x) {
                cycleStack = true;
            } else if(gamepad1.b) {
                cycleStack = false;
            }

            //OUTSIDE CYCLE OR INSIDE?
            if(gamepad1.a) {
                outsideRoute = true;
                routeOffsetY = -26;
                routeWait = 0.5;
            } else if(gamepad1.y) {
                outsideRoute = false;
                routeOffsetY = 0;
                routeWait = 0;
                waitBool = false; //no time to wait on outside route
                waitDuration = 0;
            }

            //WAIT ON ALLIANCE?
            if(gamepad1.left_bumper) {
                waitBool = true;
                waitDuration = 1;
            } else if(gamepad1.right_bumper) {
                waitBool = false;
                waitDuration = 0;
            }
            telemetry.addData("-- BLUE FAR AUTO --","");
            telemetry.addData("","");
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.addData("Cycle Stack?: ", cycleStack);
            telemetry.addData("Outside Route?: ", outsideRoute);
            telemetry.addData("Wait on Partner?: ", waitBool);
            telemetry.addData("","");

            telemetry.addData("Press X to CYCLE, B to NOT CYCLE","");
            telemetry.addData("Press A for INSIDE ROUTE, Y for OUT","");
            telemetry.addData("Press LB to WAIT, RB to ZOOM ZOOM","");
            telemetry.update();
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPosX, startPosY, startHeading));

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagHeading = tagMid;
                tagScoreOffsetY = 0;
                spikeMarkOffsetY = 0;

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreOffsetY = 5;
                spikeMarkOffsetY = -1;

            } else {
                tagHeading = tagRight;
                tagScoreOffsetY = -5.5;
                spikeMarkOffsetY = -1;
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1, robot.spikeScore())
                    .afterTime(1.25, robot.fingerHome())
                    .afterTime(1.5, robot.home())

                    //SCORE BACKDROP PIXEL
                    .afterTime(7.5, robot.low())
                    .afterTime(8.5, robot.mid())
                    .afterTime(9.0, robot.retract())

                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(55 + spikeMarkOffsetY, tagHeading)
                    .waitSeconds(0.75)

                    //DRIVE OUTSIDE TURN & DRIVE TO BACKBOARD
                    .turnTo(startHeading) //required to keep robot moving straight
                    .lineToYLinearHeading(20, startHeading)
                    .strafeToLinearHeading(new Vector2d(24, 20), tagScoreHeading)
                    .strafeToLinearHeading(new Vector2d(36, tagScorePoxY + tagScoreOffsetY), tagScoreHeading) //tagScorePosY should be dead center on backdrop. tune until it is.

                    //PUSH IN AND SCORE
                    .waitSeconds(0.01) //wait needed between strafe and line movement to seperate direction. roadrunner auto merges the two for some reason
                    //.turnTo(tagScoreHeading) //if the wait doesn't work try this. remove if not needed
                    //.lineToXLinearHeading(tagScorePosX, tagScoreHeading) //if the wait and turn doesn't work, try this. remove if not needed
                    .lineToX(tagScorePosX) //this should be a vertical move into the back drop. watch dashboard and make sure this is the case

                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            //CYCLE PIXEL STACK
            if (cycleStack) {
                Action pixelCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.5 + routeWait, robot.intakeLevel5())
                        .afterTime(4.5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(5 + routeWait, robot.transfer())
                        .afterTime(6.5 + routeWait, robot.intakeStop())
                        .afterTime(7 + (routeWait * 2) + waitDuration, robot.mid())
                        .afterTime(9 + routeWait + waitDuration, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + 1 + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.5 + waitDuration)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading)
                        .waitSeconds(1)
                        .build();


                Actions.runBlocking(pixelCycle1);
                drive.updatePoseEstimate();

                Action pixelCycle2 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.5 + routeWait, robot.intakeGround())
                        .afterTime(5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(5.5 + routeWait, robot.transfer())
                        .afterTime(6.5 + routeWait, robot.intakeStop())
                        .afterTime(7 + (routeWait * 2), robot.mid())
                        .afterTime(9 + routeWait, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.5)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX + cycleScoreOffsetX, cycleScorePosY), tagScoreHeading)
                        .waitSeconds(1)
                        .build();


                Actions.runBlocking(pixelCycle2);
                drive.updatePoseEstimate();
            }

            //PARK THE ROBOT
            Action parkBot = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    .afterTime(0, robot.home())

                    //MOVEMENT ---------------------------------------------------------------------
                    .strafeToLinearHeading(new Vector2d(parkPosX, parkPosY), tagScoreHeading)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(parkBot);
            drive.updatePoseEstimate();
            break;
        }
    }
}