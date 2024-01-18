package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedClose extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public boolean waitBool = false;
    public int waitDuration; //how long to wait on partner alliance in seconds

    //START POS
    double startPosX = 12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startPosY = -72 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(90);

    //PRELOAD POS
    double spikeMarkPosY = -45;
    double spikeMarkOffsetY; //change spike mark tape forward movement
    double tagHeading;
    double tagLeft = Math.toRadians(150);
    double tagMid = Math.toRadians(100);
    double tagRight = Math.toRadians(60);

    double tagScorePosX = 46; //center preload tag score pos X
    double tagScorePoxY = -57; //center preload tag score pos Y
    double tagScoreOffsetY; //controls left-right preload displacement
    double tagScoreHeading = Math.toRadians(0);


    double routeOffsetY; //how far from center tag to move for outside cycle run
    double routeWait; //need more time for outside route

    //PARK POS
    double parkPosX = 48;
    double parkPosY = -71;

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
        telemetry.update();

        Actions.runBlocking(new SequentialAction( //init robot
                robot.initRobot()
        ));

        sleep(1000);

        Actions.runBlocking(new SequentialAction( //turn off slider to prevent magic smoke
                robot.stopMotors()
        ));

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            //WAIT ON ALLIANCE?
            if(gamepad1.left_bumper) {
                waitBool = true;
                waitDuration = 1;
            } else if(gamepad1.right_bumper) {
                waitBool = false;
                waitDuration = 0;
            }

            telemetry.addData("-- RED CLOSE AUTO --","");
            telemetry.addData("","");
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.addData("Wait on Partner?: ", waitBool);
            telemetry.addData("","");

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
                tagScoreOffsetY = 0;
                spikeMarkOffsetY = -1;

            } else {
                tagHeading = tagRight;
                tagScoreOffsetY = 0;
                spikeMarkOffsetY = -5;
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.place())
                    .afterTime(1.75, robot.openRightClaw())
                    .afterTime(2.5, robot.closeRightClaw())
                    .afterTime(2.5, robot.score())

                    //SCORE BACKDROP PIXEL
                    .afterTime(5, robot.place())
                    .afterTime(6, robot.openLeftClaw())
                    .afterTime(6.75, robot.closeLeftClaw())
                    .afterTime(6.75, robot.score())


                    //MOVEMENT ---------------------------------------------------------------------
                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(spikeMarkPosY + spikeMarkOffsetY, startHeading)
                    .turnTo(tagHeading)
                    .waitSeconds(1.75)

                    //TURN TO BACKBOARD
                    .strafeToLinearHeading(new Vector2d(tagScorePosX, tagScorePoxY + tagScoreOffsetY), tagScoreHeading)
                    .waitSeconds(2)

                    //PUSH IN AND SCORE
                    //.lineToX(tagScorePosX)
                    //.waitSeconds(1)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            if(waitBool){
                sleep(waitDuration * 1000);
            }

            //PARK THE ROBOT
            Action parkBot = drive.actionBuilder(drive.pose)
                    //ACTIONS ----------------------------------------------------------------------
                    .afterTime(0, robot.initRobot())
                    .afterTime(1, robot.stopMotors())

                    //MOVEMENT ---------------------------------------------------------------------
                    .strafeToLinearHeading(new Vector2d(parkPosX, parkPosY), tagScoreHeading)
                    .waitSeconds(2)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(parkBot);
            drive.updatePoseEstimate();
            break;
        }
    }
}