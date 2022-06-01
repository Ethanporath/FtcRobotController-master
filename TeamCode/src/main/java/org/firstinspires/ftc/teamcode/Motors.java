package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor ls;

    public Motors(HardwareMap hardwareMap) {
        fl = hardwareMap.dcMotor.get("FrontLeft");
        bl = hardwareMap.dcMotor.get("BackLeft");
        fr = hardwareMap.dcMotor.get("FrontRight");
        br = hardwareMap.dcMotor.get("BackRight");
        ls = hardwareMap.dcMotor.get("linearslide");

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
    public enum Linearslide {
        out(-.5),
        in(.5),
        stop(0);

        public final double value;

        private Linearslide(double value) {
            this.value = value;
        }
    }

    public void Linearslidego(double distance, double power) {
        ls.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double circumfrence = 3.14*1.6;//pi * diameter
        double rotationsNeeded =distance/circumfrence;
        int encoderDrivingTarget = (int)(rotationsNeeded*560);
        ls.setTargetPosition(encoderDrivingTarget);

        ls.setPower(power);

        ls.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (ls.isBusy()) {

        }
        //stop the motor
        ls.setPower(0);

    }


}
