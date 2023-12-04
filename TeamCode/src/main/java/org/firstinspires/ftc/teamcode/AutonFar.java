package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutonFar extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    public String fieldSide = "None";
    public boolean cycleStack = true;

    public boolean waitBool = false;
    public double waitDuration = 2.0; //how long to wait on partner alliance! 

    //START POS
    double startPosX = 12;
    double startPosY = 72;
    double startHeading = Math.toRadians(90);

    //TAG POS
    double tagHeading = Math.toRadians(90);
    double tagOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPosX, startPosY, startHeading));
        drive.updatePoseEstimate();

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

            //WAIT ON ALLIANCE?
            if(gamepad1.left_bumper) {
                cycleStack = true;
            } else if(gamepad1.right_bumper) {
                cycleStack = false;
            }

            telemetry.addData("-- FAR AUTO --","");
            telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
            telemetry.addData("Side: ", fieldSide);
            telemetry.addData("Cycle Stack?: ", cycleStack);
            telemetry.addData("Wait on Partner?: ", waitBool);
            telemetry.addData("","");

            telemetry.addData("Press X for BLUE, B for RED","");
            telemetry.addData("Press A to CYCLE, Y to NOT CYCLE","");
            telemetry.addData("Press LB to WAIT, RB to VROOM VROOM","");
            telemetry.update();
        }

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {

            //SELECT TEAM ELEMENT SIDE
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagHeading = Math.toRadians(100);
            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagHeading = Math.toRadians(100); //130
            } else {
                tagHeading = Math.toRadians(100); //60
            }

            //SCORE SPIKE MARK PIXEL & DRIVE TO BACKDROP
            Action spikeMark = drive.actionBuilder(drive.pose)
                    //SCORE MARK PIXEL
                    .afterTime(0, robot.spikeExtend())
                    .afterTime(1, robot.spikeScore())
                    .afterTime(1.25, robot.fingerHome())
                    .afterTime(1.5, robot.home())

                    //PREPARE BACKDROP PIXEL
                    .afterTime(3, robot.low())

                    //THE HEADING IS CONTROLLED BY THE VISION CODE
                    .lineToYLinearHeading(55, tagHeading)
                    .waitSeconds(0.5)
                    .splineToLinearHeading(new Pose2d(36, 38, Math.toRadians(180)), Math.toRadians(tagHeading))
                    .build();

            Actions.runBlocking(spikeMark);
            drive.updatePoseEstimate();

            //SCORE BACKDROP PIXEL
            Action backDrop = drive.actionBuilder(drive.pose)
                    //CLEAR PIXEL AFTER SCORING
                    .afterTime(0.5, robot.mid())

                    //PUSH INTO BACKDROP & SCORE
                    .lineToX(44)
                    .build();

            Actions.runBlocking(backDrop);
            drive.updatePoseEstimate();

            //CYCLE PIXEL STACK
            Action cyclePixel = drive.actionBuilder(drive.pose)
                    //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                    .afterTime(0, robot.home())
                    .afterTime(1.75, robot.intakeLevel5())
                    .afterTime(3.25, robot.intakeUp())
                    .afterTime(4, robot.transfer())
                    .afterTime(5, robot.mid())

                    //GOTO STACK AND RETURN
                    .lineToX(-55)
                    .waitSeconds(0.75)
                    .lineToX(42)
                    .waitSeconds(0.5)
                    .build();

            Actions.runBlocking(cyclePixel);
            drive.updatePoseEstimate();

            Action cyclePixel1 = drive.actionBuilder(drive.pose)
                    //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                    .afterTime(0.5, robot.home())
                    .afterTime(1.75, robot.intakeGround())
                    .afterTime(3.25, robot.intakeUp())
                    .afterTime(4, robot.transfer())
                    .afterTime(5, robot.mid())

                    //GOTO STACK AND RETURN
                    .lineToX(-55)
                    .waitSeconds(0.75)
                    .lineToX(42)
                    .waitSeconds(0.5)
                    .build();

            Actions.runBlocking(cyclePixel1);
            drive.updatePoseEstimate();

            Action cyclePixel2 = drive.actionBuilder(drive.pose)
                    //TUCK IN SCORE BUCKET & WHIP OUT INTAKE
                    .afterTime(0.5, robot.home())
                    .afterTime(1.75, robot.intakeGround())
                    .afterTime(3.25, robot.intakeUp())
                    .afterTime(4, robot.transfer())
                    .afterTime(5, robot.mid())
                    .afterTime(7, robot.home())

                    //GOTO STACK AND RETURN
                    .lineToX(-55)
                    .waitSeconds(0.75)
                    .lineToX(42)
                    .waitSeconds(2)
                    .build();

            Actions.runBlocking(cyclePixel2);
            drive.updatePoseEstimate();
            break;
        }
    }
}