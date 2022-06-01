package org.firstinspires.ftc.teamcode.PurePursuit;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;



public class RobotMovement {
    public RobotMovement (HardwareMap hardwareMap) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Autos auto = new Autos(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoint, Point robotLocation, double followRadius, SampleMecanumDrive drive) {
        CurvePoint followMe = new CurvePoint(pathPoint.get(0));

        for (int i=0; i < pathPoint.size()-1; i++) {
            CurvePoint startLine = pathPoint.get(i);
            CurvePoint endLine = pathPoint.get(i+1);

            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotLocation,followRadius,startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000000;

            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y -drive.getPoseEstimate().getY(), thisIntersection.x -drive.getPoseEstimate().getX());
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - Math.toRadians(drive.getPoseEstimate().getHeading())));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

        }
        return followMe;
    }
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle,SampleMecanumDrive drive) {
        CurvePoint followMe = getFollowPointPath(allPoints,new Point(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()),allPoints.get(0).followDistance,drive);

        goToPosition(followMe.x,followMe.y,followMe.moveSpeed,followAngle,followMe.turnSpeed,drive);

    }

    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     * @param drive
     */
    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed,SampleMecanumDrive drive) {


        double StartX = drive.getPoseEstimate().getX();
        double StartY = drive.getPoseEstimate().getY();
        double StartT = Math.toRadians(drive.getPoseEstimate().getHeading());

        double distanceToTarget = Math.hypot(x-StartX, y-StartY);

        double absoluteAngleToTarget = Math.atan2(y-StartY,x-StartX);

        double relativeAngleToPoint =  MathFunctions.AngleWrap(absoluteAngleToTarget-(StartT-Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint)*distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint)*distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;


        drive.setDrivePower(new Pose2d(movementXPower*movementSpeed, movementYPower*movementSpeed, Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1)*turnSpeed));

        if (distanceToTarget < 10) {
            drive.setDrivePower(new Pose2d(movementXPower*movementSpeed, movementYPower*movementSpeed, 0));


        }


    }

}
