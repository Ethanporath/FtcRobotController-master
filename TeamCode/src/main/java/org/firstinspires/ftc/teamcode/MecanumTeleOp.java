package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    public enum ServoHeights {
        collect(0.75),
        dump(0.20);
        public final double value;

        private ServoHeights(double value) {
            this.value = value;
        }
    }

    public enum ServoCollect {
        in(1),
        out(-1),
        Hold(0);
        public final double value;

        private ServoCollect(double value) {
            this.value = value;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our hardware
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        Servo armleft = hardwareMap.servo.get("armleft");
        Servo armright = hardwareMap.servo.get("armright");
        CRServo handleft = hardwareMap.crservo.get("handleft");
        CRServo handright = hardwareMap.crservo.get("handright");

        //Declaring our Enums
        ServoHeights arm = ServoHeights.collect;
        ServoCollect hand = ServoCollect.in;




        // Reverse the right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Gamepad Controls
        double lx,ly,rx;

        //Motor Power
        double backRight, frontLeft, frontRight, backLeft, denominator;

        telemetry.addLine("hi");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            lx = gamepad1.left_stick_x * 1.1;
            ly = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            if (gamepad1.a) arm = ServoHeights.collect;
            if (gamepad1.b) arm = ServoHeights.dump;
            if (gamepad1.left_bumper) hand = ServoCollect.out;
            else if (gamepad1.right_bumper) hand = ServoCollect.in;
            else hand = ServoCollect.Hold;


            armleft.setPosition(arm.value);
            armright.setPosition(1-arm.value);
            handleft.setPower(hand.value);
            handright.setPower(-hand.value);

            denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);
            frontLeft = (ly - lx + rx) / denominator;
            backLeft = (ly + lx + rx) / denominator;
            frontRight = (ly - lx - rx) / denominator;
            backRight = (ly + lx - rx) / denominator;

            motorFrontLeft.setPower(frontLeft);
            motorBackLeft.setPower(backLeft);
            motorFrontRight.setPower(frontRight);
            motorBackRight.setPower(backRight);


        }

    }
}