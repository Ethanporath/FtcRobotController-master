package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Autos {
/*
    private static MotionProfile generateProfile(boolean movingForward, double DISTANCE) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    public Autos(HardwareMap hardwareMap) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    }


    public void DRIVE(double distance, SampleMecanumDrive drive, boolean forwards) {
        NanoClock clock = NanoClock.system();
        double profileStart = clock.seconds();
        boolean movingForwards = true;
        double StartX = drive.getPoseEstimate().getX();
        double StartY = drive.getPoseEstimate().getY();
        double StartT = drive.getPoseEstimate().getHeading();
        MotionProfile activeProfile = generateProfile(true, distance - StartX);

        while (!isStopRequested()) {
            double profileTime = clock.seconds() - profileStart;
            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards, distance - StartX);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
            Pose2d poseEstimate = drive.getPoseEstimate();

            double theta = drive.getPoseEstimate().getHeading();
            double headingError = Math.min(2 * Math.PI - theta, theta) / (Math.PI);
            if (theta < 0) {
                headingError *= -1;
            }
            if (forwards) {
                headingError *= -1;
            }
            double yError = (StartY - drive.getPoseEstimate().getY()) / 10;

            drive.setDrivePower(new Pose2d(targetPower, yError, headingError));
            drive.updatePoseEstimate();

            double xError = (StartX - drive.getPoseEstimate().getX()) / 10;

            if (forwards && (drive.getPoseEstimate().getX() >= distance || targetPower <= 0)) {
                drive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }
            if (!forwards && (drive.getPoseEstimate().getX() <= distance || targetPower >= 0)) {
                drive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }
            Telemetry.Item Startx = telemetry.addData("StartX", StartX);
            Telemetry.Item TargetPower = telemetry.addData("TargetPower", targetPower);
            Telemetry.Item Xpostition = telemetry.addData("X", drive.getPoseEstimate().getX());
            Telemetry.Item Heading = telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        }

    }
/*
    public void STRAFE(double distance, SampleMecanumDrive drive, boolean left) {
        NanoClock clock = NanoClock.system();
        double profileStart = clock.seconds();
        boolean movingForwards = true;
        double StartY = drive.getPoseEstimate().getY();
        double StartX = drive.getPoseEstimate().getX();
        MotionProfile activeProfile = generateProfile(true, (distance - StartY) * 1.85);


        while (!isStopRequested()) {
            double profileTime = clock.seconds() - profileStart;
            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards, (distance - StartY) * 1.85);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

            double theta = drive.getPoseEstimate().getHeading();
            double headingError = Math.min(2 * Math.PI - theta, theta) / (Math.PI);
            if (theta < 0) {
                headingError *= -1;
            }
            if (!left) {
                headingError *= -1;
            }
            double xError = (StartX - drive.getPoseEstimate().getX()) / 10;


            drive.setDrivePower(new Pose2d(xError, targetPower, headingError));
            drive.updatePoseEstimate();

            if (left && (drive.getPoseEstimate().getY() <= (distance) || targetPower >= 0)) {
                drive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }
            if (!left && (drive.getPoseEstimate().getY() >= (distance) || targetPower <= 0)) {
                drive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }
            Pose2d poseEstimate = drive.getPoseEstimate();
            Telemetry.Item Starty = telemetry.addData("StartY", StartY);
            Telemetry.Item TargetPower = telemetry.addData("TargetPower", targetPower);
            Telemetry.Item Yposition = telemetry.addData("Y", drive.getPoseEstimate().getY());
            Telemetry.Item Heading = telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        }
    }

    public void TURN(double Degrees, SampleMecanumDrive drive) {
        TrajectorySequence ts = drive.trajectorySequenceBuilder(turn)
                .turn(Math.toRadians(Degrees)) // Turns 45 degrees counter-clockwise
                .build();

        drive.followTrajectorySequence(ts);
    }
*/
}
