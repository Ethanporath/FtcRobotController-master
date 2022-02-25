package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class test5 extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addLine("hi");
            telemetry.update();
        }
    }
}