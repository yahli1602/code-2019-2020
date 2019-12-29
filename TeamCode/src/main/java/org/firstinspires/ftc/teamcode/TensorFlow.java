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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
public class TensorFlow extends LinearOpMode {

    public DcMotor rdrive1 = null;
    public DcMotor rdrive2 = null;
    public DcMotor ldrive1 = null;
    public DcMotor ldrive2 = null;
    public DcMotor slide = null;
    public DcMotor elevator = null;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

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


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AVHZDTL/////AAABmQcZurBiA01smn3EpdcPCJpZqB8HZL60ujXKBU3ejemhikdsno1L3+7QKhYWSXEfUl5uWZxBqPJXl6Qj0AG3XKuq/jLKmyLJ67xHlYM/LoVKbxhjxGJJ5stO+21qtYET0KberI6XObNkTmskQ8kLQX7QwLhmllfyhu25bPFWwmVdnGq3jRAxoCNKP9ktqKkqp62Fl39qcvOwCOBPqG0uFMFHwVaNavRHS1f4fnuZXk4QqEDo5e2K9J/sCR/2BvvzdPV3QfTkUPNm/8dfW2nsxCM2E9rpj67CFq9fOAHjY+7tp4o2U/yJbxc5RBr5mZ9/CeQk7zfl9rQv7WrVWevfvHqvb2xMsoqVJGze9rE62AmI";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        /*rdrive1 = hardwareMap.get(DcMotor.class, "rDrive1");
        rdrive2 = hardwareMap.get(DcMotor.class, "rDrive2");
        ldrive1 = hardwareMap.get(DcMotor.class, "lDrive1");
        ldrive2 = hardwareMap.get(DcMotor.class, "lDrive2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        rdrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rdrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ldrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        rdrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rdrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        ldrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        ldrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD);


        if (opModeIsActive()) {
        int z = 0;
        int x = 0;

            while (opModeIsActive() && x==0) {

                rdrive1.setPower(0.65);
                rdrive2.setPower(0.65);
                ldrive1.setPower(0.95);
                ldrive2.setPower(0.95);
                sleep(600);
                rdrive1.setPower(0);
                rdrive2.setPower(0);
                ldrive1.setPower(0);
                ldrive2.setPower(0);
                sleep(100);
                ldrive1.setPower(-0.25);
                ldrive2.setPower(-0.25);
                sleep(60);
                ldrive1.setPower(0);
                ldrive2.setPower(0);


                x++;
            }
            if (tfod != null) {
                tfod.activate();
            }


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
                        if(recognition.getLabel() == "skystone" || recognition.getLabel() == LABEL_SECOND_ELEMENT){
                            canSeeSkystone = true;
                        }

                      }
                      if (updatedRecognitions.size() == 3){
                          seeSkystone = -1;
                          seeStone1 = -1;
                          seeStone2 = -1;

                          if (seeSkystone == -1 && seeStone1 == -1 && seeStone2 == -1){

                              if (updatedRecognitions.get(0).equals(LABEL_SECOND_ELEMENT)){
                                  skyStoneX = updatedRecognitions.get(0).getLeft();
                              }else if (updatedRecognitions.get(0).equals(LABEL_FIRST_ELEMENT)){
                                  Stone1X = updatedRecognitions.get(0).getLeft();
                              }


                              if (updatedRecognitions.get(1).equals(LABEL_SECOND_ELEMENT)){
                                  skyStoneX = updatedRecognitions.get(1).getLeft();

                              }else if (Stone1X != 0){

                                      Stone2X = updatedRecognitions.get(1).getLeft();
                              }else if(Stone1X == 0){
                                      Stone1X = updatedRecognitions.get(1).getLeft();
                              }

                              if (updatedRecognitions.get(2).equals(LABEL_SECOND_ELEMENT)){
                                  skyStoneX = updatedRecognitions.get(2).getLeft();
                              }else if (updatedRecognitions.get(2).equals(LABEL_FIRST_ELEMENT)){
                                  Stone2X = updatedRecognitions.get(2).getLeft();
                              }


                              if (skyStoneX < Stone1X && skyStoneX < Stone2X){
                                  skystonePostion = -1;
                              }else if (skyStoneX > Stone1X && skyStoneX > Stone2X){
                                  skystonePostion = 1;
                              }else if (skyStoneX > Stone1X && skyStoneX < Stone2X || skyStoneX < Stone1X && skyStoneX > Stone2X) {
                                  skystonePostion = 0;
                              }
                              telemetry.addData("skyStone position",skystonePostion);
                          }


                      }
                      if (updatedRecognitions.size() == 2){
                          seeSkystone = -1;
                          seeStone1 = -1;

                          if (seeSkystone == -1 && seeStone1 == -1){
                              for (Recognition object : updatedRecognitions){
                                  if (object.getLabel().equals(LABEL_SECOND_ELEMENT)){
                                      skyStoneX = object.getLeft();
                                  }else if (object.getLabel().equals(LABEL_FIRST_ELEMENT)){
                                      Stone1X = object.getLeft();
                                  }

                              }
                              if (skyStoneX < Stone1X){
                                  skystonePostion = -1;
                                  Stone1Postion = 1;

                              }else{
                                  skystonePostion = 1;
                                  Stone1Postion = -1;
                                  //TODO: add a driving function to the sky ston that actualy is a skystone and a stone

                              }

                              telemetry.addData("skyStone position",skystonePostion);
                              telemetry.addData("Stone1X",Stone1X);
                              telemetry.addData("Stone2X",Stone2X);


                          }

                          telemetry.addData("skyStone position",skystonePostion);
                          telemetry.addData("Stone1X",Stone1X);
                          telemetry.addData("Stone2X",Stone2X);

                      }
                      if (updatedRecognitions.equals(LABEL_SECOND_ELEMENT)){
                          telemetry.addData("can see skyStone","start laasof");
                      }




                      telemetry.update();
                    }

                    if (canSeeSkystone && z==0){
                        telemetry.addData("can see skystone","start osef");

                        ldrive1.setPower(0.25);
                        ldrive2.setPower(0.25);
                        sleep(100);
                        ldrive1.setPower(0);
                        ldrive2.setPower(0);
                        sleep(100);
                        rdrive1.setPower(0.7);
                        rdrive2.setPower(0.7);
                        ldrive1.setPower(0.7);
                        ldrive2.setPower(0.7);
                        sleep(170);
                        ldrive1.setPower(0);
                        ldrive2.setPower(0);
                        rdrive1.setPower(0);
                        rdrive2.setPower(0);
                        sleep(70);

                        z++;
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.60;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void goToStoneX(int stonePlace){
        if (stonePlace == 0){

        }else if(stonePlace == 1){

        }
    }

    private void goToStoneY(){

    }

}
