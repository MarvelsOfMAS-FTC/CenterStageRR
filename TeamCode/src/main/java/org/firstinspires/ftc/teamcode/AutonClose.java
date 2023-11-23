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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class AutonClose extends LinearOpMode {
    //VARIABLES---------------------------------------------------------------------------------------------------------------
    double startposx = 12;
    double startposy = 72;
    double startheading = Math.toRadians(90);
    double tagposx=0;
    double tagposy=5;
    double tagheading = 1;

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
        robot.home(1);
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
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(startposy+tagposy)
                            .turn(tagheading)
                            .build());
            telemetry.addLine("0");
            robot.extend(550);
            robot.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extend.setPower(1);
            sleep(1000);
            telemetry.addLine("1");
            telemetry.update();
            robot.extend.setPower(0);
            robot.low();
            robot.climbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbl.setPower(1);
            robot.climbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.climbr.setPower(1);
            sleep(1000);
            telemetry.addLine("2");
            telemetry.update();
            robot.climbl.setPower(0);
            robot.climbr.setPower(0);
            telemetry.addLine("3");
            telemetry.update();
            robot.home(1);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0,startposy+tagposy,0))
                            .turn(-tagheading)
                            .lineToY(startposy)
                            .build());
            break;


        }


    }

}

