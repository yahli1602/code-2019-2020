/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous.imageProsessing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "TensorFlow Webcam only", group = "TensorFlow")
public class TensorFlow_Webcam_11229 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    //motors setup

    //driving motors
    /*private DcMotor rDrive1 = null;
    private DcMotor rDrive2 = null;
    private DcMotor lDrive1 = null;
    private DcMotor lDrive2 = null;
    private DcMotor slide = null;
    //teleop_11226_A
    private DcMotor teleop_11226_A = null;
    //fold collection
    private DcMotor foldcollect = null;
    //collection
    private Servo collectRight = null;
    private Servo collectLeft = null;
    //grabbing the build plate
    private Servo grabber1 = null;
    private Servo grabber2 = null;
    private TouchSensor stoneIn = null;*/


    private double skyStoneX = 0;
    private double Stone1X = 0;
    private double Stone2X = 0;
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
    public void runOpMode() {

        /*rDrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rDrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        lDrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        lDrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        teleop_11226_A = hardwareMap.get(DcMotor.class, "teleop_11226_A");
        foldcollect = hardwareMap.get(DcMotor.class, "foldCollect");
        collectRight = hardwareMap.get(Servo.class, "collectRight");
        collectLeft = hardwareMap.get(Servo.class, "collectLeft");
        grabber1 = hardwareMap.get(Servo.class, "grabber1");
        grabber2 = hardwareMap.get(Servo.class, "grabber2");
        stoneIn = hardwareMap.get(TouchSensor.class, "cubeIn");*/


        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        if (tfod != null) {
            tfod.activate();
        }


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        waitForStart();

        /*rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.FORWARD);
        lDrive2.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        teleop_11226_A.setDirection(DcMotor.Direction.FORWARD);
        foldcollect.setDirection(DcMotor.Direction.FORWARD);
        grabber1.setDirection(Servo.Direction.FORWARD);
        grabber2.setDirection(Servo.Direction.REVERSE);

        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        teleop_11226_A.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        teleop_11226_A.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        teleop_11226_A.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/


        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            telemetry.addData("hight:", recognition.getTop() - recognition.getBottom());


                        }

                        //find out the skystone location if it sees 3 cubes
                        if (updatedRecognitions.size() > 0) {
                            skystonePostion = seeThreeObj(updatedRecognitions);
                        }



                                /*if (updatedRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)){
                                    skyStoneX = updatedRecognitions.get(0).getLeft();
                                }else if (updatedRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)){
                                    Stone1X = updatedRecognitions.get(0).getLeft();
                                }


                                if (updatedRecognitions.get(1).getLabel().equals(LABEL_SECOND_ELEMENT)){
                                    skyStoneX = updatedRecognitions.get(1).getLeft();

                                }else if (updatedRecognitions.get(1).getLabel().equals(LABEL_FIRST_ELEMENT)) {

                                    if (Stone1X != 0) {

                                        Stone2X = updatedRecognitions.get(1).getLeft();
                                    } else if (Stone1X == 0) {
                                        Stone1X = updatedRecognitions.get(1).getLeft();
                                    }
                                }

                                if (updatedRecognitions.get(2).getLabel().equals(LABEL_SECOND_ELEMENT)){
                                    skyStoneX = updatedRecognitions.get(2).getLeft();
                                }else if (updatedRecognitions.get(2).getLabel().equals(LABEL_FIRST_ELEMENT)){
                                    Stone2X = updatedRecognitions.get(2).getLeft();
                                }


                                if (skyStoneX < Stone1X && skyStoneX < Stone2X){
                                    skystonePostion = 1;
                                }else if (skyStoneX > Stone1X && skyStoneX > Stone2X){
                                    skystonePostion = 3;
                                }else if (skyStoneX > Stone1X && skyStoneX < Stone2X || skyStoneX < Stone1X && skyStoneX > Stone2X) {
                                    skystonePostion = 2;
                                }*/
                        telemetry.addData("skyStone position", skystonePostion);
                        telemetry.addData("skyStoneX:", skyStoneX);
                        telemetry.addData("Stone1X:", Stone1X);
                        telemetry.addData("Stone2X:", Stone2X);


                        //hopefully find out the skystone location if it sees only 2 stones(it has a chance of 2:1 secesseding
                        /*if (updatedRecognitions.size() == 2){
                            if (updatedRecognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)){
                                skyStoneX = updatedRecognitions.get(0).getLeft();
                            }else if (updatedRecognitions.get(0).getLabel().equals(LABEL_FIRST_ELEMENT)){
                                Stone1X = updatedRecognitions.get(0).getLeft();
                            }

                            if (updatedRecognitions.get(1).getLabel().equals(LABEL_SECOND_ELEMENT)){
                                skyStoneX = updatedRecognitions.get(1).getLeft();
                            }else if (updatedRecognitions.get(1).getLabel().equals(LABEL_FIRST_ELEMENT)){
                                Stone1X = updatedRecognitions.get(1).getLeft();
                            }

                            if (skyStoneX < Stone1X){
                                skystonePostion = 1;
                            }else if (skyStoneX > Stone1X) {
                                skystonePostion = 3;

                            }

                            telemetry.addData("skyStone position",skystonePostion);
                            telemetry.addData("skyStoneX:",skyStoneX);
                            telemetry.addData("Stone1X:",Stone1X);

                        }*/


                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    //init the Vuforia
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
        tfodParameters.minimumConfidence = 0.45;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //if see 3 diffrent objects
    private int seeThreeObj(List<Recognition> Recognitions){

        int lastPosition = 0;

        int skyStoneP = 0;

        double skyStoneX = 0;




        if (Recognitions.get(0).getLabel().equals(LABEL_SECOND_ELEMENT)) {
            skyStoneX = Recognitions.get(0).getLeft();
        }

        if (Recognitions.size() == 2){
            if (Recognitions.get(1).getLabel().equals(LABEL_SECOND_ELEMENT)) {
                skyStoneX = Recognitions.get(1).getLeft();
            }
        }
        if (Recognitions.size() == 3){
            if (Recognitions.get(2).getLabel().equals(LABEL_SECOND_ELEMENT)){
                skyStoneX = Recognitions.get(2).getLeft();
            }
        }






        if (skyStoneX < 8){
            skyStoneP = 3;
        }
        else if (skyStoneX < 140){
            skyStoneP = 1;
        }else if (skyStoneX > 320){
            skyStoneP = 3;
        }else{
            skyStoneP = 2;
        }
        telemetry.addData("skyStoneX",skyStoneX);

        return skyStoneP;
    }
}