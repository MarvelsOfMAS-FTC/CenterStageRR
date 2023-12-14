package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedFar extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public boolean cycleStack = true;

    public boolean outsideRoute = true;

    public boolean waitBool = false;
    public int waitDuration; //how long to wait on partner alliance in seconds

    //START POS
    double startPosX = -36 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startPosY = -72 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(270);

    //PRELOAD POS
    double spikeMarkPosY = -55;
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double tagHeading;
    double tagLeft = Math.toRadians(120.5 + 180);
    double tagMid = Math.toRadians(94 + 180);
    double tagRight = Math.toRadians(75 + 180);

    double tagScorePosX = 48; //center preload tag score pos X
    double tagScorePoxY = -42; //center preload tag score pos Y -- THIS NEEDS TO BE DEAD CENTER FOR MIDDLE PLACEMENT ON BACK DROP --
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreHeading = Math.toRadians(180);


    //CYCLING POS
    double pixelStackPosX = -55.15; //how far into back wall to drive
    double pixelStackOffsetX = -2.5;
    double pixelStackPosY = -42;
    double pixelStackOffsetY = -3;

    double cycleScorePosX = 46; //push in more than tag score
    double cycleScoreOffsetX = 1;
    double cycleScorePosY = -44; //used to dodge right pixel on
    double cycleScoreOffsetY = 0;

    double routeOffsetY  = 25; //how far from center tag to move for outside cycle run
    double routeWait = 0.5; //need more time for outside route

    //PARK POS
    double parkPosX = 55;
    double parkPosY = -18;

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
            telemetry.addData("-- RED FAR AUTO --","");
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
                tagScoreOffsetY = -45;
                spikeMarkOffsetY = -1.5;//positive towards wall
            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreOffsetY = -40;
                spikeMarkOffsetY = 1;
            } else {
                tagHeading = tagRight;
                tagScoreOffsetY = -50.5;
                spikeMarkOffsetY = -10;
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1.3, robot.spikeUp())
                    .afterTime(1.6, robot.spikeScore())
                    .afterTime(2, robot.fingerHome())
                    .afterTime(2.5, robot.home())

                    //SCORE BACKDROP PIXEL


                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(spikeMarkPosY + spikeMarkOffsetY, tagHeading)
                    .waitSeconds(1.1)//keep this 1.75

                    //DRIVE OUTSIDE TURN & DRIVE TO BACKBOARD
                    .turnTo(Math.toRadians(269)) //required to keep robot moving straight
                    .lineToY(-20)

                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            Action score = drive.actionBuilder(drive.pose)
                    .afterTime(3.5, robot.low())
                    .afterTime(6, robot.mid())
                    .afterTime(6.5,robot.retract())
                    .afterTime(6.7,robot.home())
                    .strafeToLinearHeading(new Vector2d(34, cycleScorePosY + routeOffsetY), tagScoreHeading)
                    .waitSeconds(0.01)
                    .strafeTo(new Vector2d(40, tagScoreOffsetY))
                    .strafeTo(new Vector2d(48, tagScoreOffsetY))
                    .endTrajectory()
                    .build();
            Actions.runBlocking(score);

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
                        .afterTime(5.5 + routeWait, robot.transfer())
                        .afterTime(6.4 + routeWait, robot.intakeStop())
                        .afterTime(6.5 + (routeWait * 2), robot.mid())
                        .afterTime(9.5 + routeWait, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(36, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + 2, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.lastHope)
                        //.strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.fast)
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
                        .afterTime(6.5 + routeWait, robot.transfer())
                        .afterTime(7.5 + routeWait, robot.intakeStop())
                        .afterTime(7.5 + routeWait, robot.mid())
                        .afterTime(9.5 + routeWait, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(36, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading,drive.lastHope)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + pixelStackOffsetY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.5)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.lastHope)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.lastHope)
                        //.strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.fast)
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

            //Actions.runBlocking(parkBot);
            drive.updatePoseEstimate();
            break;
        }
    }
}