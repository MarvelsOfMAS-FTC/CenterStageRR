package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BlueClose extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public String fieldSide = "Blue";
    public boolean cycleStack = true;

    public boolean insideRoute = true;

    public boolean waitBool = false;
    public int waitDuration; //how long to wait on partner alliance in seconds. set below.

    //START POS
    double startPosX = 12;
    double startPosY = 72;
    double startHeading = Math.toRadians(90);

    //TAG POS
    double tagHeading;
    double tagLeft = Math.toRadians(120);
    double tagMid = Math.toRadians(100);
    double tagRight = Math.toRadians(72);
    double tagScoreYOffset = 5.5; //Controls Left/Right Displacement. Absolute Value

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
            } else if(gamepad1.y) {
                insideRoute = false;
            }

            //WAIT ON ALLIANCE?
            if(gamepad1.left_bumper) {
                waitBool = true;
                waitDuration = 2;
            } else if(gamepad1.right_bumper) {
                waitBool = false;
                waitDuration = 0;
            }

            telemetry.addData("-- BLUE CLOSE AUTO --","");
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
                tagScoreYOffset = 0; //Center Pos. Offset not required

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreYOffset = tagScoreYOffset * 1; //+Y for Left

            } else {
                tagHeading = tagRight;
                tagScoreYOffset = tagScoreYOffset * -1; //-Y for Right
            }

            //SCORE PRELOAD PIXELS
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1, robot.spikeScore())
                    .afterTime(1.25, robot.fingerHome())
                    .afterTime(1.5, robot.home())

                    //SCORE BACKDROP PIXEL
                    .afterTime(3.5, robot.low())
                    .afterTime(4.5, robot.mid())
                    .afterTime(5.0, robot.home())


                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(54.5, tagHeading)
                    .waitSeconds(0.75)

                    //DRIVE TO BACKBOARD
                    .strafeToLinearHeading(new Vector2d(27, 42 + tagScoreYOffset), Math.toRadians(180))
                    .turnTo(Math.toRadians(180))

                    //PUSH AND SCORE
                    .lineToX(43)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            //CYCLE PIXEL STACK
            if (cycleStack) {
                if (insideRoute){
                    Action inCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                            //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                            .afterTime(2.5, robot.intakeLevel5())
                            .afterTime(4.5, robot.intakeUp())
                            .afterTime(5, robot.transfer())
                            .afterTime(6 + waitDuration, robot.mid())
                            .afterTime(8 + waitDuration, robot.home())
                            .afterTime(8, robot.intakeStop())

                            //CENTER ROBOT
                            .strafeToLinearHeading(new Vector2d(25, 44), Math.toRadians(180))
                            .waitSeconds(0.1)

                            //GOTO STACK AND WAIT IF NEEDED
                            .lineToX(-55)
                            .waitSeconds(0.75 + waitDuration)

                            //RETURN AND SCORE
                            .lineToX(44)
                            .waitSeconds(1)
                            .build();

                    Actions.runBlocking(inCycle1);
                    drive.updatePoseEstimate();

                    Action inCycle2 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                            //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                            .afterTime(2, robot.intakeGround())
                            .afterTime(4, robot.intakeUp())
                            .afterTime(4.5, robot.transfer())
                            .afterTime(5.5, robot.mid())
                            .afterTime(7.5, robot.home())
                            .afterTime(7.5, robot.intakeStop())

                            //GOTO STACK
                            .lineToX(-58)
                            .waitSeconds(0.75)

                            //RETURN AND SCORE
                            .lineToX(44)
                            .waitSeconds(2)
                            .build();

                    Actions.runBlocking(inCycle2);
                    drive.updatePoseEstimate();

                }
                else{

                }
            }

        break;
        }
    }
}