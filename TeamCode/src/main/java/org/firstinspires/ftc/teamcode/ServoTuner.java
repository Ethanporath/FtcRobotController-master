package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        double left = 0.355;
        double right = 0.65;

        boolean isPressed = false;
        boolean isLeft = true;

        Servo armleft = hardwareMap.servo.get("armleft");
        Servo armright = hardwareMap.servo.get("armright");

        waitForStart();
        while (opModeIsActive()) {
            armleft.setPosition(left);
            armright.setPosition(right);

            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                isLeft = !isLeft;
            }
            if (gamepad1.a) {
                isPressed = false;
            }

            if (!isPressed) {
                if (gamepad1.dpad_up && isLeft) {
                    left += 0.01;
                    isPressed = true;
                } else if (gamepad1.dpad_down && isLeft) {
                    left -= 0.01;
                    isPressed = true;
                } else if (gamepad1.dpad_up && !isLeft) {
                    right += 0.01;
                    isPressed = true;
                } else if (gamepad1.dpad_down && !isLeft) {
                    right -= 0.01;
                    isPressed = true;
                }
            }


            telemetry.addData("Left", left);
            telemetry.addData("Right", right);
            telemetry.addData((isLeft ? "Left" : "Right"), isLeft);

            telemetry.update();
        }
    }

}