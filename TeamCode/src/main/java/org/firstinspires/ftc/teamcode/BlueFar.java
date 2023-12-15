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
    public boolean cycleStack = true;

    public boolean outsideRoute = true;

    public boolean waitBool = false;
    public int waitDuration; //how long to wait on partner alliance in seconds

    //START POS
    double startPosX = -36 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startPosY = 72 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(90);

    //PRELOAD POS
    double spikeMarkPosY = 55;
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double tagHeading;
    double tagLeft = Math.toRadians(120);
    double tagMid = Math.toRadians(94);
    double tagRight = Math.toRadians(76);

    double tagScorePosX = 48; //center preload tag score pos X
    double tagScorePoxY = 42; //center preload tag score pos Y -- THIS NEEDS TO BE DEAD CENTER FOR MIDDLE PLACEMENT ON BACK DROP --
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreHeading = Math.toRadians(180);


    //CYCLING POS
    double pixelStackPosX = -55; //how far into back wall to drive
    double pixelStackOffsetX = -2.5;
    double pixelStackPosY = 43;
    double pixelStackOffsetY = 0;

    double cycleScorePosX = 49; //push in more than tag score
    double cycleScoreOffsetX = 0;
    double cycleScorePosY = 44; //used to dodge right pixel on transit
    double cycleScoreOffsetY = -0.5;


    double routeOffsetY  = -25; //how far from center tag to move for outside cycle run
    double routeWait = 0.5; //need more time for outside route

    //PARK POS
    double parkPosX = 55;
    double parkPosY = 18;

    double spikeWait = 0.0;

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
                tagScoreOffsetY = 0.5;
                spikeMarkOffsetY = 1.75;//positive towards wall

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreOffsetY = 8.5;
                spikeMarkOffsetY = 9;

            } else {
                tagHeading = tagRight;
                tagScoreOffsetY = -6;
                spikeMarkOffsetY = 1;
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1.2, robot.spikeUp())
                    .afterTime(1.5, robot.spikeScore())
                    .afterTime(2, robot.fingerHome())
                    .afterTime(2.5, robot.home())

                    //SCORE BACKDROP PIXEL
                    .afterTime(8.5 + waitDuration, robot.low())
                    .afterTime(10 + waitDuration, robot.mid())
                    .afterTime(10.5 + waitDuration, robot.retract())

                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(spikeMarkPosY + spikeMarkOffsetY, tagHeading)
                    .waitSeconds(1.75)//keep this 1.75

                    //DRIVE OUTSIDE TURN & DRIVE TO BACKBOARD
                    .turnTo(startHeading-0.0001) //required to keep robot moving straight
                    .lineToYLinearHeading(20, startHeading)
                    .strafeToLinearHeading(new Vector2d(30, 20), tagScoreHeading)
                    .waitSeconds(waitDuration)
                    .strafeToLinearHeading(new Vector2d(38, tagScorePoxY + tagScoreOffsetY), tagScoreHeading) //tagScorePosY should be dead center on backdrop. tune until it is.

                    //PUSH IN AND SCORE
                    .waitSeconds(0.01) //wait needed between strafe and line movement to seperate direction. roadrunner auto merges the two for some reason
                    .turnTo(tagScoreHeading+0.00001) //if the wait doesn't work try this. remove if not needed
                    .lineToXLinearHeading(tagScorePosX, tagScoreHeading) //if the wait and turn doesn't work, try this. remove if not needed
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
                        .afterTime(1, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(3 + routeWait, robot.intakeLevel5())
                        .afterTime(4.9 + routeWait, robot.intakeGround())
                        .afterTime(5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(5.75 + routeWait, robot.transfer())
                        .afterTime(6.5 + routeWait, robot.intakeStop())
                        .afterTime(6.5 + (routeWait * 2), robot.mid())
                        .afterTime(8.6 + routeWait, robot.retract())


                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.lastHope)
                        .build();


                Actions.runBlocking(pixelCycle1);
                drive.updatePoseEstimate();

                Action pixelCycle2 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.5 + routeWait, robot.intakeGround())
                        .afterTime(5.5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(6.25 + routeWait, robot.transfer())
                        .afterTime(7 + routeWait, robot.intakeStop())
                        .afterTime(7.1 + routeWait, robot.mid())
                        .afterTime(8.5 + routeWait, robot.retract())


                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + cycleScoreOffsetY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX + 2, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + pixelStackOffsetY + routeOffsetY), tagScoreHeading)
                        //.waitSeconds(0.5)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX+2, cycleScorePosY), tagScoreHeading, drive.lastHope)
                      //  .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading)
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