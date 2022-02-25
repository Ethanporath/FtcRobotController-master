package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    public Motors(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("FrontLeft");
        bl = hardwareMap.dcMotor.get("BackLeft");
        fr = hardwareMap.dcMotor.get("FrontRight");
        br = hardwareMap.dcMotor.get("BackRight");

        // Reverse the right side motors
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Drive(double power) {
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);
    }

    public void Strafe(double power) {
        fl.setPower(-power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(-power);
    }

    public void Turn(double power) {
        fl.setPower(-power);
        bl.setPower(-power);
        fr.setPower(power);
        br.setPower(power);
    }

    public void Stop() {
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }


}
