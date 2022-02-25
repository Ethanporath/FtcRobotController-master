package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "camera test", group = "Concept")
public class Cameratest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Gold Mineral";
    private static final String LABEL_SECOND_ELEMENT = "Silver Mineral";

    private static final String VUFORIA_KEY =
            "ARR2q2v/////AAABmYP6HYbAY0KijmQ7l2bLtLpk7NpVYLcN+YDUCj+fOO+G8RXF74+Ax9+nOvHkwnnFEXrptyCia3jwT6d6iit5DS5LLI9ziRuJKIvJ8U3ZtKzAaMsfZMGLFXj/J6jJy6QrsYvQXxDhtwT7H3/MdKyzSxYJCZJAs6cPxNfh3Ca38VlVPjYj+H2CGkys1bV3O22w7mpc1WGpmrjgojxcksm2WQqvKT7LM1MdhJSwloDRCjDUgAggG6Fjhfh2Vjq0RPxQwyKRiRF4z8riOwyHv5t0rs5S+0CgjDjn0Xu9B40PokH+LDzYJPzoaVtXRRh8tGfPYs/vfTLZI6z5rYZHWiIkpyWZDnNDqeZFqA5Uzn+lqdk5";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private boolean left = false;
    private boolean center = false;
    private boolean right = false;

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Servo armleft = hardwareMap.servo.get("armleft");
        Servo armright = hardwareMap.servo.get("armright");

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

        }
        armleft.setPosition(Servos.ServoHeights.scanLeft.value);
        armright.setPosition(Servos.ServoHeights.scanRight.value);
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            armleft.setPosition(Servos.ServoHeights.scanLeft.value);
            armright.setPosition(Servos.ServoHeights.scanRight.value);

            while (runtime.seconds() <= 2.0 && !left && !center) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 1) {
                            if (updatedRecognitions.get(0).getTop() < 200) {
                                left = true;
                                telemetry.addLine("left");
                            } else {
                                center = true;
                                telemetry.addLine("center");
                            }
                            break;
                        }
                    }
                }
            }
        }
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
