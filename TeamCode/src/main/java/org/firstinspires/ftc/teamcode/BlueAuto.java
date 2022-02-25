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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

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



@Config
@Autonomous(name = "BlueAuto", group = "Concept")
public class BlueAuto extends LinearOpMode {

    private static MotionProfile generateProfile(boolean movingForward, double DISTANCE) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Gold Mineral";
    private static final String LABEL_SECOND_ELEMENT = "Silver Mineral";
    private static final String VUFORIA_KEY =
            "ARR2q2v/////AAABmYP6HYbAY0KijmQ7l2bLtLpk7NpVYLcN+YDUCj+fOO+G8RXF74+Ax9+nOvHkwnnFEXrptyCia3jwT6d6iit5DS5LLI9ziRuJKIvJ8U3ZtKzAaMsfZMGLFXj/J6jJy6QrsYvQXxDhtwT7H3/MdKyzSxYJCZJAs6cPxNfh3Ca38VlVPjYj+H2CGkys1bV3O22w7mpc1WGpmrjgojxcksm2WQqvKT7LM1MdhJSwloDRCjDUgAggG6Fjhfh2Vjq0RPxQwyKRiRF4z8riOwyHv5t0rs5S+0CgjDjn0Xu9B40PokH+LDzYJPzoaVtXRRh8tGfPYs/vfTLZI6z5rYZHWiIkpyWZDnNDqeZFqA5Uzn+lqdk5";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private boolean Left = false;
    private boolean Center = false;
    private boolean Right = false;


    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo armleft = hardwareMap.servo.get("armleft");
        Servo armright = hardwareMap.servo.get("armright");
        CRServo intake = hardwareMap.crservo.get("hand");
        CRServo Ducks = hardwareMap.crservo.get("duck");


        Servos.ServoHeights leftArm = Servos.ServoHeights.collectLeft;
        Servos.ServoHeights rightArm = Servos.ServoHeights.collectRight;
        Servos.ServoCollect hand = Servos.ServoCollect.in;
        Servos.ServoCollect duck = Servos.ServoCollect.in;

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

        }
        armleft.setPosition(Servos.ServoHeights.initLeft.value);
        armright.setPosition(Servos.ServoHeights.initRight.value);
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
           armleft.setPosition(Servos.ServoHeights.scanLeft.value);
            armright.setPosition(Servos.ServoHeights.scanRight.value);
            Thread.sleep(2000);

            while (runtime.seconds() <= 2.0 && !Right && !Center) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 1) {
                            if (updatedRecognitions.get(0).getTop() > 200) {
                                Right = true;
                                telemetry.addData("right", updatedRecognitions.get(0).getTop());
                            } else {
                                Center = true;
                                telemetry.addData("center", updatedRecognitions.get(0).getTop());
                            }
                            break;
                        }
                    }
                }

            }


            if (tfod != null) {
                tfod.shutdown();
            }

            if (!Right && !Center) {
                Left = true;
                telemetry.addLine("left");
            }

            telemetry.update();

            if (Left == true) {
                rightArm = Servos.ServoHeights.dumpRightbottom;
                leftArm = Servos.ServoHeights.dumpLeftbottom;
                telemetry.addLine("Bottom");

            } else if (Right == true) {
                rightArm = Servos.ServoHeights.dumpRighttop;
                leftArm = Servos.ServoHeights.dumpLefttop;
                telemetry.addLine("Top");
            } else {
                rightArm = Servos.ServoHeights.dumpRightmid;
                leftArm = Servos.ServoHeights.dumpLeftmid;
                telemetry.addLine("Right");
            }

            armleft.setPosition(leftArm.value);
            armright.setPosition(rightArm.value);

            DRIVE(-2,drive,false);
            STRAFE(-18, drive, true); //Left 0,-16
            Ducks.setPower(duck.out.value);
            Thread.sleep(9000);
            Ducks.setPower(duck.Hold.value);
            STRAFE(0, drive,false);
            Thread.sleep(250);
            DRIVE(-14, drive,false ); //-21,30
            Thread.sleep(250);
            STRAFE(30, drive, false);// - 0,30
            Thread.sleep(250);
            drive.updatePoseEstimate();
            if (drive.getPoseEstimate().getY() > 30) STRAFE(30, drive, false);// - 0,30
            else if (drive.getPoseEstimate().getY() < 30) STRAFE(30, drive, true);// - 0,30
            if (Right)DRIVE(-21.5, drive,false ); //-21,30
            if (Center) DRIVE(-20, drive,false ); //-22.5,30
            if (Left) DRIVE(-18.5, drive,false ); //-24,30

            intake.setPower(Servos.ServoCollect.out.value);
            Thread.sleep(2500);
            intake.setPower(Servos.ServoCollect.Hold.value);
            intake.setPower(Servos.ServoCollect.in.value);
            rightArm = Servos.ServoHeights.intakeRight;
            leftArm = Servos.ServoHeights.intakeLeft;
            armleft.setPosition(leftArm.value);
            armright.setPosition(rightArm.value);
            Thread.sleep(2000);

            //THIS IS THE START OF CODE FOR A SECOND BLOCK!!!!!
            DRIVE(-4, drive, true);
            Thread.sleep(250);
            intake.setPower(Servos.ServoCollect.Hold.value);
            rightArm = Servos.ServoHeights.dumpRighttop;
            leftArm = Servos.ServoHeights.dumpLefttop;
            armleft.setPosition(leftArm.value);
            armright.setPosition(rightArm.value);
            Thread.sleep(1000);
            DRIVE(-23,drive,false);
            Thread.sleep(250);
            intake.setPower(Servos.ServoCollect.out.value);
            Thread.sleep(2500);
            intake.setPower(Servos.ServoCollect.Hold.value);
            //THIS IS THE END OF CODE FOR A SECOND BLOCK!!!!!!

            //THIS IS THE START OF THE CODE FOR DRIVING INTO THE WAREHOUSE
//            TURN(drive);
//            drive.setDrivePower(new Pose2d(0.8, 0, 0));
//            Thread.sleep(1500);
//            drive.setDrivePower(new Pose2d(0, 0, 0));
            //THIS IS THE START OF THE CODE FOR DRIVING INTO THE WAREHOUSE


            while (!isStopRequested()) {
                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("finalX", poseEstimate.getX());
                telemetry.addData("finalY", poseEstimate.getY());
                telemetry.addData("finalHeading", Math.toRadians(poseEstimate.getHeading()));
                telemetry.update();
            }
        }
    }



    public void DRIVE(double distance, SampleMecanumDrive drive, boolean forwards) {
        NanoClock clock = NanoClock.system();
        double profileStart = clock.seconds();
        boolean movingForwards = true;
        double StartX = drive.getPoseEstimate().getX();
        double StartY = drive.getPoseEstimate().getY();
        MotionProfile activeProfile = generateProfile(true, distance-StartX);

        while (!isStopRequested()) {
            double profileTime = clock.seconds() - profileStart;
            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards, distance-StartX);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
            Pose2d poseEstimate = drive.getPoseEstimate();

            double theta = drive.getPoseEstimate().getHeading();
            double headingError = Math.min(2*Math.PI-theta,theta)/(Math.PI);
            if (theta<0) {
                headingError *= -1;
            }
            if (forwards) {
                headingError *= -1;
            }
            double yError = (StartY-drive.getPoseEstimate().getY())/10;

            drive.setDrivePower(new Pose2d(targetPower, yError, headingError));
            drive.updatePoseEstimate();

            double xError = (StartX-drive.getPoseEstimate().getX())/10;

            if (forwards && (drive.getPoseEstimate().getX() >= distance || targetPower <= 0)) {
                drive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }
            if (!forwards && (drive.getPoseEstimate().getX() <= distance || targetPower >= 0)) {
                drive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }
            telemetry.addData("StartX", StartX);
            telemetry.addData("TargetPower", targetPower);
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }

    }



    public void STRAFE(double distance, SampleMecanumDrive drive, boolean left) { ;
        NanoClock clock = NanoClock.system();
        double profileStart = clock.seconds();
        boolean movingForwards = true;
        double StartY = drive.getPoseEstimate().getY();
        double StartX = drive.getPoseEstimate().getX();
        MotionProfile activeProfile = generateProfile(true, (distance-StartY) *1.85);


        while (!isStopRequested()) {
            double profileTime = clock.seconds() - profileStart;
            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards, (distance-StartY) *1.85);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

            double theta = drive.getPoseEstimate().getHeading();
            double headingError = Math.min(2*Math.PI-theta,theta)/(Math.PI);
            if (theta<0) {
                headingError *= -1;
            }
            if (!left) {
                headingError *= -1;
            }
            double xError = (StartX-drive.getPoseEstimate().getX())/10;


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
            telemetry.addData("StartY", StartY);
            telemetry.addData("TargetPower", targetPower);
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }
    public void TURN (SampleMecanumDrive drive){
        while  ((Math.toDegrees(drive.getPoseEstimate().getHeading())<90 || Math.toDegrees(drive.getPoseEstimate().getHeading())>270) && !isStopRequested()) {
            double theta = drive.getPoseEstimate().getHeading();
            if (theta>1.5*Math.PI) {
                double headingError = Math.max((theta-2*Math.PI-theta)/(Math.PI),0.2);
                drive.setDrivePower(new Pose2d(0, 0, 1.5*headingError));
            }
            if (theta<0.5*Math.PI) {
                double headingError = Math.max((theta+0.5 * Math.PI) / (Math.PI),0.2);
                drive.setDrivePower(new Pose2d(0, 0, 1.5*headingError));
            }
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d(0, 0, 0));
    }



    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}