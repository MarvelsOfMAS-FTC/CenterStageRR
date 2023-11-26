package org.firstinspires.ftc.teamcode;
import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.hardware.HardwareBuffer;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class AutonClose extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    double startposx = 9;
    double startposy = 72;
    double startheading = Math.toRadians(90);
    double tagposx=0;
    double tagposy=-5;
    double tagheading = Math.toRadians(1);

    double elbowHome = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        //ROBOT INITIALIZATION ---------------------------------------------------------------------
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);

        robot.telemetry = this.telemetry;
        robot.parent = this;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startposx, startposy, startheading));

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("--Frostbite Close Auto--", true);
        telemetry.addData("Placement: ", robot.visionProcessor.getSelection());
        telemetry.update();
        robot.climbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.climbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.home();
        robot.finger.setPosition(0.37);

        //EXECUTE ACTIONS -----------------------------------------------------------------
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            tagposy = -5;
            //if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.MIDDLE) {
            //    tagheading = 1;
            //} else if (robot.visionProcessor.getSelection() == FirstVisionProcessor.Selected.LEFT) {
            //    tagheading=30;
            //} else {
            //    tagheading=-30;
            //}
            robot.elbowl.setPosition(0.29 + elbowHome);//  INTAKE UP // Transfer
            robot.elbowr.setPosition(0.25 + elbowHome);

            //DRIVE TO SCORE SPIKE
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(58)
                            .turn(tagheading)
                            .build());
            robot.spikeExtend(510);
            sleep(500);
            robot.home();

            //SPLINE TO BACKBOARD PREP POS
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0,58, Math.toRadians(90)))
                            .splineToLinearHeading(new Pose2d(36, 32, Math.toRadians(180)), Math.toRadians(90))
                            .build());
            robot.low();

            //DRIVE INTO BACKBOARD TO SCORE
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(36, 32, Math.toRadians(180)))
                            .lineToX(53)
                            .waitSeconds(1)
                            .lineToX(45)
                            .build());
            robot.mid();
            robot.home();

            //DRIVE TO PUCK STACK
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(45, 32, Math.toRadians(180)))
                            .lineToX(-60)
                            .build());

            break;


        }


    }

}

