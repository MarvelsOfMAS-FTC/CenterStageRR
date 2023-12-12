package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedCloseFast extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public boolean cycleStack = true;

    public boolean insideRoute = true;

    public boolean waitBool = false;
    public int waitDuration; //how long to wait on partner alliance in seconds

    //START POS
    double startPosX = 12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startPosY = -72 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(270);

    //PRELOAD POS
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double tagHeading;
    double tagLeft = Math.toRadians(129+180);
    double tagMid = Math.toRadians(110+180);
    double tagRight = Math.toRadians(72+180);

    double tagScorePosX = 43; //center preload tag score pos X
    double tagScorePoxY = -42; //center preload tag score pos Y
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreHeading = Math.toRadians(180);


    //CYCLING POS
    double pixelStackPosX = -55; //how far into back wall to drive
    double pixelStackOffsetX = -2.5;
    double pixelStackPosY = -40;
    double pixelStackOffsetY = 2;
    double cycleScorePosX = 48; //push in more than tag score
    double cycleScoreOffsetX = -11;
    double cycleScorePosY = -44; //used to dodge right pixel on transit

    double routeOffsetY; //how far from center tag to move for outside cycle run
    double routeWait; //need more time for outside route

    //PARK POS
    double parkPosX = 46;
    double parkPosY = -68;

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

            //INSIDE CYCLE OR OUTSIDE?
            if(gamepad1.a) {
                insideRoute = true;
                routeOffsetY = 0;
                routeWait = 0;
            } else if(gamepad1.y) {
                insideRoute = false;
                routeOffsetY = 26.5;
                routeWait = 0.5;

                waitBool = false; //no time to wait on outside route
                waitDuration = 0;
            }

            //WAIT ON ALLIANCE?
            if(gamepad1.left_bumper) {
                waitBool = true;
                waitDuration = 2;
            } else if(gamepad1.right_bumper) {
                waitBool = false;
                waitDuration = 0;
            }

            telemetry.addData("-- RED CLOSE FAST AUTO --","");
            telemetry.addData("","");
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.addData("Cycle Stack?: ", cycleStack);
            telemetry.addData("Inside Route?: ", insideRoute);
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
                spikeMarkOffsetY = 2;

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreOffsetY = 5;
                spikeMarkOffsetY = -5.6;

            } else {
                tagHeading = tagRight;
                tagScoreOffsetY = -5.2;
                spikeMarkOffsetY = 4;
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(0.75, robot.fingerScore())
                    .afterTime(1.5, robot.spikeScore())
                    .afterTime(1.75, robot.fingerHome())
                    .afterTime(2, robot.home())

                    //SCORE BACKDROP PIXEL
                    .afterTime(5, robot.low())
                    .afterTime(5.7, robot.mid())
                    .afterTime(6.5, robot.retract())

                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(-55 + spikeMarkOffsetY, tagHeading, drive.speedyVelConstraint)
                    .waitSeconds(2.25)

                    //TURN TO BACKBOARD
                    .strafeToLinearHeading(new Vector2d(27, tagScorePoxY + tagScoreOffsetY), tagScoreHeading, drive.speedyVelConstraint, drive.fastAccelConstraint)
                    .turnTo(Math.toRadians(180))

                    //PUSH IN AND SCORE
                    .lineToX(tagScorePosX)
                    .waitSeconds(0.01)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            if(waitBool){
                sleep(waitDuration * 1000);
            }

            //CYCLE PIXEL STACK
            if (cycleStack) {
                Action pixelCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.5 + routeWait, robot.intakeLevel5())
                        .afterTime(3.7 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(4.5 + routeWait, robot.transfer())
                        .afterTime(5.4 + routeWait, robot.intakeStop())
                        .afterTime( 4.9 + routeWait, robot.mid())
                        .afterTime(5.7 + routeWait, robot.retract())
                        .afterTime(6.0 + routeWait, robot.home())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.speedyVelConstraint, drive.fastAccelConstraint)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-51.5, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.speedyVelConstraint, drive.fastAccelConstraint)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading, drive.slowerVelConstraint)
                        .waitSeconds(0.5)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(54, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.speedyVelConstraint, drive.fastAccelConstraint)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.slightlySlowerVelConstraint)
                        .waitSeconds(1.2)
                        .build();


                Actions.runBlocking(pixelCycle1);
                drive.updatePoseEstimate();
                Actions.runBlocking(pixelCycle1);

                Action pixelCycle2 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2 + routeWait, robot.intakeGround())
                        .afterTime(4.5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(5.3 + routeWait, robot.transfer())
                        .afterTime(6.3 + routeWait, robot.intakeStop())
                        .afterTime(5.8 + (routeWait * 2), robot.mid())
                        .afterTime(9 + routeWait, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.fasterVelConstraint, drive.fastAccelConstraint)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + pixelStackOffsetY + routeOffsetY), tagScoreHeading, drive.slowerVelConstraint)
                        .waitSeconds(0.75)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(54, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.speedyVelConstraint, drive.fastAccelConstraint)
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
