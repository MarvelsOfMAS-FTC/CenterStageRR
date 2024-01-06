package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TestBuildRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
        telemetry.update();

        //WAIT FOR START CODE ----------------------------------------------------------------------
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("-- BLUE CLOSE AUTO --","");
            telemetry.addData("","");
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.update();
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Cam Place: ", robot.visionProcessor.getSelection());
            telemetry.update();
            if(gamepad1.a){
                Actions.runBlocking(new SequentialAction(
                        robot.openRightClaw()
                ));
            }
            if(gamepad1.b){
                Actions.runBlocking(new SequentialAction(
                        robot.openLeftClaw()
                ));
            }
            if(gamepad1.x){
                Actions.runBlocking(new SequentialAction(
                        robot.closeLeftClaw()
                ));
            }
            if(gamepad1.y){
                Actions.runBlocking(new SequentialAction(
                        robot.CloseRightClaw()
                ));
            }
            if (gamepad1.dpad_up){
                Actions.runBlocking(new SequentialAction(
                        robot.Low()
                ));
            }
            if (gamepad1.dpad_down){
                Actions.runBlocking(new SequentialAction(
                        robot.lowHome()
                ));
            }
            if (gamepad1.dpad_left){
                Actions.runBlocking(new SequentialAction(
                        robot.score()
                ));
            }
            if (gamepad1.dpad_right){
                Actions.runBlocking(new SequentialAction(
                        robot.place()
                ));
            }
            telemetry.addData("Wrist : ",robot.wrist.getPosition());
            telemetry.addData("LOW : ",robot.slider.getCurrentPosition());
            telemetry.addData("LeftClaw : ",robot.leftclaw.getPosition());
            telemetry.addData("RightClaw : ",robot.rightclaw.getPosition());
            telemetry.addData("Arm : ",robot.arm.getCurrentPosition());
            telemetry.update();
        }

    }
}
