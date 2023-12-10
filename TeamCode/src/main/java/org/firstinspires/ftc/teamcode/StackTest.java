package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class StackTest extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------

    //START POS
    double startPosX = -48 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startPosY = -12 + 0; //MODIFY OFFSET TO CALIBRATE IN COMPETITION
    double startHeading = Math.toRadians(180);


    //CYCLING POS
    double pixelStackPosX = -55; //how far into back wall to drive
    double pixelStackOffsetX = -2.5;
    double pixelStackPosY = -41;
    double cycleScorePosX = -48; //push in more than tag score
    double cycleScoreOffsetX = 1;
    double cycleScorePosY = -44; //used to dodge right pixel on transit


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


        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startPosX, startPosY, startHeading));

        //EXECUTE ACTIONS -----------------------------------------------------------------
        while (opModeIsActive() && !isStopRequested()) {
                Action pixelCycle1 = drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        //ACTIONS --------------------------------------------------------------
                        //RETRACT
                        .afterTime(0, robot.home())
                        .afterTime(0.1, robot.intakeLevel5())

                        //GOTO STACK AND WAIT IF NEEDED
                      //  .strafeToLinearHeading(new Vector2d(-48, startPosY), Math.toRadians(180))
                        .waitSeconds(3) //added to make approach more gentle
                       // .strafeToLinearHeading(new Vector2d(-72, startPosY), Math.toRadians(180))
                        .lineToX(-72, drive.slowerVelConstraint)
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(-48, startPosY), Math.toRadians(180))
                        .build();

                Actions.runBlocking(pixelCycle1);
                drive.updatePoseEstimate();
            break;
        }
    }
}
