package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutonFar extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    double startposx = 0;
    double startposy = 0;
    double startheading = Math.toRadians(90);
    double tagposx=0;
    double tagposy=-5;
    double tagheading = Math.toRadians(0);

    double elbowHome = (0.0);

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //INIT POSITIONS
        //robot.home(1);
        robot.finger.setPosition(0.37);
        //robot.elbowl.setPosition(elbowHome + .32);//  INTAKE UP // Transfer
        //robot.elbowr.setPosition((.28 + elbowHome));
        //robot.wrist.setPosition(0.28);

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("--Frostbite Close Auto--", true);
        telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //EXECUTE ACTIONS -----------------------------------------------------------------
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
                tagheading = 0;
            } else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
                tagheading= 90;
            } else {
                tagheading= -90;
            }

            Action drive_to_drop = drive.actionBuilder(drive.pose)
                    .lineToY(28)
                    .endTrajectory().build();

            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            drive_to_drop
                    ),
                    new SequentialAction(
                            robot.groundWrist(),
                            robot.low(530)
                    )
                )
            );
            drive.updatePoseEstimate();
            break;
        }
    }

}

