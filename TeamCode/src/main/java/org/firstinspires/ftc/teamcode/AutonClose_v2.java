package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonClose_v2 extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public String fieldSide = "Blue";
    public boolean cycleStack = true;

    public boolean insideRoute = false;

    public boolean waitBool = false;
    public int waitDuration = 1000; //how long to wait on partner alliance in ms!

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

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPosX, startPosY, startHeading));

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            //SIDE SELECT
            if(gamepad1.x) {
                fieldSide = "Blue";
                startHeading = Math.toRadians(90);
            } else if(gamepad1.b) {
                fieldSide = "Red";
            }

            //CYCLE PIXELS SELECT
            if(gamepad1.a) {
                cycleStack = true;
            } else if(gamepad1.y) {
                cycleStack = false;
            }

            //INSIDE CYCLE OR OUTSIDE?
            if(gamepad1.left_bumper) {
                insideRoute = true;
            } else if(gamepad1.right_bumper) {
                insideRoute = false;
            }

            //WAIT ON ALLIANCE?
            if(gamepad1.left_trigger > 0.1) {
                waitBool = true;
            } else if(gamepad1.right_trigger > 0.1) {
                waitBool = false;
            }

            telemetry.addData("-- FAR AUTO --","");
            telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
            telemetry.addData("Side: ", fieldSide);
            telemetry.addData("Cycle Stack?: ", cycleStack);
            telemetry.addData("Inside Route?: ", insideRoute);
            telemetry.addData("Wait on Partner?: ", waitBool);
            telemetry.addData("","");

            telemetry.addData("Press X for BLUE, B for RED","");
            telemetry.addData("Press A to CYCLE, Y to NOT CYCLE","");
            telemetry.addData("Press LB for IN, RB to OUT","");
            telemetry.addData("Press LT to WAIT, RT to ZOOM","");
            telemetry.update();
        }

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagHeading = tagMid;
                tagScoreYOffset = 0; //Center Pos. Offset not required

            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = tagLeft;
                tagScoreYOffset = tagScoreYOffset * 1; //Add Some Y for Left

            } else {
                tagHeading = tagRight;
                tagScoreYOffset = tagScoreYOffset * -1; //Remove Y for Right
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
                    .afterTime(5, robot.home())

                    //DRIVE TO SPIKE MARK
                    .lineToYLinearHeading(55, tagHeading)
                    .waitSeconds(0.75)

                    //DRIVE TO BACKBOARD
                    .strafeToLinearHeading(new Vector2d(27, 42 + tagScoreYOffset), Math.toRadians(180))
                    .turnTo(Math.toRadians(180))

                    //PUSH AND SCORE
                    .lineToX(43)
                    .waitSeconds(5)
                    .endTrajectory()
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();
            break;

//            //WAIT FOR PARTNER
//            if (waitBool) {
//                sleep(waitDuration);
//            }
//
//            //CYCLE PIXEL STACK
//            if (cycleStack) {
//                Action cyclePixel = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
//                        //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
//                        .afterTime(0, robot.home())
//                        .afterTime(1.75, robot.intakeLevel5())
//                        .afterTime(3.25, robot.intakeUp())
//                        .afterTime(4, robot.transfer())
//                        .afterTime(5, robot.mid())
//
//                        //GOTO STACK AND RETURN
//                        .lineToX(-55)
//                        .waitSeconds(0.75)
//                        .lineToX(42)
//                        .waitSeconds(0.5)
//                        .build();
//
//                Actions.runBlocking(cyclePixel);
//                drive.updatePoseEstimate();
//            }

        }
    }
}