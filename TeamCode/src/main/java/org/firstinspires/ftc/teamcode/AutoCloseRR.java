package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoCloseRR extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public boolean cycleStack = true;

    public boolean insideRoute = true;

    public boolean waitBool = false;
    public int waitOnAlliance = 0;
    public int waitDuration = 1; //how long to wait

    public int sideMult = 1; //default blue = 1, red = -1

    //START POS
    double startPosX = 12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITIONAutoCloseRR
    double startPosY = (72 + 0) * sideMult; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(90*sideMult);

    //PRELOAD POS
    double spikeMarkPosY = 55*sideMult;
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double tagHeading;
    double tagLeft = Math.toRadians(120*sideMult);
    double tagMid = Math.toRadians(100*sideMult);
    double tagRight = Math.toRadians(66.5*sideMult);

    double tagScorePosX = 48; //center preload tag score pos X
    double tagScorePoxY = 42*sideMult; //center preload tag score pos Y
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreHeading = Math.toRadians(180*sideMult);


    //CYCLING POS
    double pixelStackPosX = -55; //how far into back wall to drive
    double pixelStackOffsetX = -2.5;
    double pixelStackPosY = 43.2*sideMult;
    double pixelStackOffsetY = -2.2*sideMult;

    double cycleScorePosX = 48; //push in more than tag score
    double cycleScoreOffsetX = 1;
    double cycleScorePosY = 44*sideMult; //used to dodge right pixel on transit

    double routeOffsetY; //how far from center tag to move for outside cycle run
    double routeWait; //need more time for outside route

    //PARK POS
    double parkPosX = 46;
    double parkPosY = 68*sideMult;
    boolean OppositePark=false;
    String Side = "Blue";

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
                routeOffsetY = 0*sideMult;
                routeWait = 0;
            } else if(gamepad1.y) {
                insideRoute = false;
                routeOffsetY = -26.5*sideMult;
                routeWait = 0.5;

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

            telemetry.addData("-- BLUE CLOSE AUTO --","");
            telemetry.addData("","");
            telemetry.addData("Inside Route?: ", insideRoute);
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.addData("Cycle Stack?: ", cycleStack);;
            telemetry.addData("Wait on Partner?: ", waitBool);
            telemetry.addData("Opposite Park : ",OppositePark);
            telemetry.addData("Side : ",Side);
            telemetry.addData("","");

            telemetry.addData("Press X to CYCLE, B to NOT CYCLE","");
            telemetry.addData("Press A for INSIDE ROUTE, Y for OUT","");
            telemetry.addData("Press LB to WAIT, RB to ZOOM ZOOM","");
            telemetry.addData("TouchPad for opposite park","");
            telemetry.addData("right trigger for blue left trigger for red...","");
            telemetry.update();
        }
        if(Side.equalsIgnoreCase("Blue")){
            sideMult = 1;
        }else{
            sideMult = -1;
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPosX, startPosY, startHeading));

        if(OppositePark){
            parkPosY=20*sideMult;
        }else{
            parkPosY=68*sideMult;
        }

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagHeading = tagMid;
                tagScoreOffsetY = 0;
                spikeMarkOffsetY = 0;

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreOffsetY = 5*sideMult;
                spikeMarkOffsetY = -1*sideMult;

            } else {
                tagHeading = tagRight;
                tagScoreOffsetY = -6.3*sideMult;
                spikeMarkOffsetY = 8.5*sideMult;
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1, robot.spikeScore())
                    .afterTime(1.5, robot.fingerHome())
                    .afterTime(2, robot.home())

                    //SCORE BACKDROP PIXEL
                    .afterTime(4, robot.low())
                    .afterTime(6, robot.mid())
                    .afterTime(6.5, robot.retract())

                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(spikeMarkPosY + spikeMarkOffsetY, tagHeading)
                    .waitSeconds(1.75+0.5) // t≅2.25

                    //TURN TO BACKBOARD
                    .strafeToLinearHeading(new Vector2d(27, tagScorePoxY + tagScoreOffsetY), tagScoreHeading)
                    .turnTo(Math.toRadians(180.0000000000001))

                    //PUSH IN AND SCORE
                    .lineToX(tagScorePosX-5)
                    .waitSeconds(0.01)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            if(waitBool){
                sleep(waitOnAlliance * 1000);
            }

            //CYCLE PIXEL STACK
            if (cycleStack) {
                Action pixelCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0.5, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.5 + routeWait, robot.intakeLevel5())
                        .afterTime(4.5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(5.25 + routeWait, robot.intakeUp())
                        .afterTime(6 + routeWait, robot.intakeStop())
                        .afterTime(6.5 + (routeWait * 2), robot.mid())
                        .afterTime(8 + routeWait, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX, pixelStackPosY + routeOffsetY), tagScoreHeading, drive.defaultVelConstraint)
                        .waitSeconds(0.75)

                        //RETURN TO BACKBOARD AND SCORE
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading, drive.defaultVelConstraint, drive.defaultAccelConstraint)
                        .waitSeconds(0.01)
                        .strafeToLinearHeading(new Vector2d(cycleScorePosX, cycleScorePosY), tagScoreHeading, drive.defaultVelConstraint)
                        .waitSeconds(1)
                        .build();


                Actions.runBlocking(pixelCycle1);
                drive.updatePoseEstimate();

                Action pixelCycle2 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, tagScoreHeading))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0.5, robot.home())

                        //WHIP OUT INTAKE & FEED
                        .afterTime(2.0 + routeWait, robot.intakeGround())
                        .afterTime(4.5 + routeWait, robot.intakeUp())

                        //TRANSFER & SCORE
                        .afterTime(5.25 + routeWait, robot.intakeUp())
                        .afterTime(6 + routeWait, robot.intakeStop())
                        .afterTime(6.5 + (routeWait * 2), robot.mid())
                        .afterTime(8.5 + routeWait, robot.retract())

                        //MOVEMENT -------------------------------------------------------------
                        //CENTER ROBOT ON PIXEL STACK
                        .strafeToLinearHeading(new Vector2d(25, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01)

                        //GOTO STACK AND WAIT IF NEEDED
                        .strafeToLinearHeading(new Vector2d(-48, cycleScorePosY + routeOffsetY), tagScoreHeading)
                        .waitSeconds(0.01) //added to make approach more gentle
                        .strafeToLinearHeading(new Vector2d(pixelStackPosX + pixelStackOffsetX, pixelStackPosY + pixelStackOffsetY + routeOffsetY), tagScoreHeading)
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
}