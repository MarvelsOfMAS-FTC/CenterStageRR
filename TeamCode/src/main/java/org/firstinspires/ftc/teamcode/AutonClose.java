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
        robot.home(1);
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
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(64)
                            .turn(tagheading)
                            .build());
            telemetry.addLine("0");
            robot.extend(510);
            robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extend.setPower(1);
            sleep(1000);
            telemetry.addLine("1");
            telemetry.update();
            robot.extend.setPower(0);
            robot.climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbl.setPower(1);
            robot.climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbr.setPower(1);
            sleep(1000);
            robot.finger.setPosition(0.84);
            telemetry.addLine("2");
            telemetry.update();
            robot.climbl.setPower(0);
            robot.climbr.setPower(0);
            telemetry.addLine("3");
            telemetry.update();
            robot.home(1);
            robot.extend.setTargetPosition(0);
            robot.extend.setPower(-0.5);
            robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            robot.extend.setPower(0);
            robot.home(1);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0,64,0))
                            .splineTo(new Vector2d(36, 32), Math.toRadians(180))
                            .build());
            robot.extend.setTargetPosition(450);
            robot.climbl.setTargetPosition(280);
            robot.climbr.setTargetPosition(280);
            robot.score.setPosition(0.38);
            robot.extend.setPower(1);
            robot.climbl.setPower(1);
            robot.climbr.setPower(1);
            robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(36, 32, Math.toRadians(180)))
                            .lineToX(53)
                            .waitSeconds(1)
                            .lineToX(45)
                            .build());
            robot.climbl.setTargetPosition(525);
            robot.climbr.setTargetPosition(525);
            robot.climbl.setPower(1);
            robot.climbr.setPower(1);
            robot.climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extend.setTargetPosition(0);
            robot.extend.setPower(-0.5);
            robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            robot.extend.setPower(0);
            robot.home(1);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(45, 32, Math.toRadians(180)))
                            .lineToX(-60)
                            .build());

            break;


        }


    }

}

