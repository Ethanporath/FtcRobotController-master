package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.List;


@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our hardware
        Motors motors = new Motors(hardwareMap);
        Servo armleft = hardwareMap.servo.get("armleft");
        Servo armright = hardwareMap.servo.get("armright");
        CRServo intake = hardwareMap.crservo.get("hand");
        CRServo Ducks = hardwareMap.crservo.get("duck");

        //Declaring our Enums
        Servos.ServoHeights leftArm = Servos.ServoHeights.collectLeft;
        Servos.ServoHeights rightArm = Servos.ServoHeights.collectRight;
        Servos.ServoCollect hand = Servos.ServoCollect.in;
        Servos.ServoCollect duck = Servos.ServoCollect.in;


        //Gamepad Controls
        double lx, ly, rx;

        //Motor Power
        double backRight, frontLeft, frontRight, backLeft, denominator;

        telemetry.addLine("hi");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            lx = gamepad1.left_stick_x;
            ly = gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            if (gamepad1.dpad_up) motors.Drive(-0.4);
            if (gamepad1.dpad_down) motors.Drive(0.4);
            if (gamepad1.dpad_left) motors.Strafe(-0.6);
            if (gamepad1.dpad_right) motors.Strafe(0.6);

            if (gamepad1.right_trigger > 0.1) {
                if (gamepad1.b) hand = Servos.ServoCollect.in;
                else if (gamepad1.left_trigger > 0.1) hand = Servos.ServoCollect.out;
                rightArm = Servos.ServoHeights.dumpRighttop;
                leftArm = Servos.ServoHeights.dumpLefttop;
            } else if (gamepad1.right_bumper) {
                hand = Servos.ServoCollect.in;
                rightArm = Servos.ServoHeights.intakeRight;
                leftArm = Servos.ServoHeights.intakeLeft;
            } else if (gamepad1.left_bumper) {
                hand = Servos.ServoCollect.out;
            } else if (gamepad1.a) {
                hand = Servos.ServoCollect.in;
                rightArm = Servos.ServoHeights.dumpRightmid;
                leftArm = Servos.ServoHeights.dumpLeftmid;
            } else if (gamepad1.y) {
                hand = Servos.ServoCollect.in;
                rightArm = Servos.ServoHeights.dumpRightbottom;
                leftArm = Servos.ServoHeights.dumpLeftbottom;
            } else {
                hand = Servos.ServoCollect.Hold;
                rightArm = Servos.ServoHeights.collectRight;
                leftArm = Servos.ServoHeights.collectLeft;
                if (gamepad1.b) hand = Servos.ServoCollect.in;
                else if (gamepad1.left_trigger > 0.1) hand = Servos.ServoCollect.out;
            }


            if (gamepad1.x) {
                duck = Servos.ServoCollect.out;
            } else if (gamepad1.b) {
                duck = Servos.ServoCollect.in;
            } else {
                duck = Servos.ServoCollect.Hold;
            }


            armleft.setPosition(leftArm.value);
            armright.setPosition(rightArm.value);
            intake.setPower(hand.value);
            Ducks.setPower(duck.value);

            denominator = 2.4;
            frontLeft = (ly + lx + rx) / denominator;
            backLeft = (ly - lx + rx) / denominator;
            frontRight = (ly - lx - rx) / denominator;
            backRight = (ly + lx - rx) / denominator;

            motors.fl.setPower(frontLeft);
            motors.bl.setPower(backLeft);
            motors.fr.setPower(frontRight);
            motors.br.setPower(backRight);

            telemetry.addData("Leftfront", frontLeft);
            telemetry.addData("Rightfront", frontRight);
            telemetry.addData("Leftback", backLeft);
            telemetry.addData("Rightback", backRight);
            telemetry.addData("Time", runtime.seconds());
            telemetry.update();

        }

    }

}