
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Auto 11229 B Dimona straight drive", group = "Autonomous")
public class Auto_11229_B_Dimona extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    ElapsedTime elapsedTime = new ElapsedTime();
    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide = null;
    public DcMotor elevator = null;

    private float skyStoneX = 0;
    private float Stone1X = 0;
    private float Stone2X = 0;
    int skystonePostion;
    int Stone1Postion;
    int Stone2Postion;
    int seeSkystone = 0;
    int seeStone1 = 0;
    int seeStone2 = 0;
    boolean canSeeSkystone = false;

    private static final String VUFORIA_KEY =
            "AVHZDTL/////AAABmQcZurBiA01smn3EpdcPCJpZqB8HZL60ujXKBU3ejemhikdsno1L3+7QKhYWSXEfUl5uWZxBqPJXl6Qj0AG3XKuq/jLKmyLJ67xHlYM/LoVKbxhjxGJJ5stO+21qtYET0KberI6XObNkTmskQ8kLQX7QwLhmllfyhu25bPFWwmVdnGq3jRAxoCNKP9ktqKkqp62Fl39qcvOwCOBPqG0uFMFHwVaNavRHS1f4fnuZXk4QqEDo5e2K9J/sCR/2BvvzdPV3QfTkUPNm/8dfW2nsxCM2E9rpj67CFq9fOAHjY+7tp4o2U/yJbxc5RBr5mZ9/CeQk7zfl9rQv7WrVWevfvHqvb2xMsoqVJGze9rE62AmI";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;




    @Override
    public void runOpMode() throws InterruptedException {
        rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rdrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ldrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ldrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (tfod != null) {
            tfod.activate();
        }

int x = 0;
        while (opModeIsActive() && x==0) {

            rdrive1.setPower(0.7);
            rdrive2.setPower(0.7);
            ldrive1.setPower(1);
            ldrive2.setPower(1);
            sleep(500);
            rdrive1.setPower(0);
            rdrive2.setPower(0);
            ldrive1.setPower(0);
            ldrive2.setPower(0);
            elevator.setPower(-1);
            sleep(1000);
            elevator.setPower(1);
            sleep(500);
            elevator.setPower(0);
            x++;
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }


    //init the TensorFlow
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //if see 3 diffrent objects
    private int seeThreeObj(List<Recognition> Recognitions){

        if (Recognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(0).getLeft();
        }else if (Recognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)){
            Stone1X = Recognitions.get(0).getLeft();
        }


        if (Recognitions.get(1).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(1).getLeft();

        }else if (Recognitions.get(1).getLabel().equals(LABEL_FIRST_ELEMENT)) {

            if (Stone1X != 0) {

                Stone2X = Recognitions.get(1).getLeft();
            } else if (Stone1X == 0) {
                Stone1X = Recognitions.get(1).getLeft();
            }
        }

        if (Recognitions.get(2).getLabel().equals(LABEL_SECOND_ELEMENT)){
            skyStoneX = Recognitions.get(2).getLeft();
        }else if (Recognitions.get(2).getLabel().equals(LABEL_FIRST_ELEMENT)){
            Stone2X = Recognitions.get(2).getLeft();
        }


        if (skyStoneX < Stone1X && skyStoneX < Stone2X){
            skystonePostion = 1;
        }else if (skyStoneX > Stone1X && skyStoneX > Stone2X){
            skystonePostion = 3;
        }else if (skyStoneX > Stone1X && skyStoneX < Stone2X || skyStoneX < Stone1X && skyStoneX > Stone2X) {
            skystonePostion = 2;
        }

        return skystonePostion;
    }
}




