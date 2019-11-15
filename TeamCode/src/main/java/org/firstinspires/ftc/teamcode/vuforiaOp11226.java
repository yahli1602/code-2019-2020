package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class vuforiaOp11226 {

    float degreesToTurn;


    //DcMotor rDrive1 = hardwareMap.get(DcMotor.class, "testMotor");
    //DcMotor rDrive2 = hardwareMap.get(DcMotor.class, "testMotor");
    //DcMotor lDrive1 = hardwareMap.get(DcMotor.class, "testMotor");
    //DcMotor lDrive2 = hardwareMap.get(DcMotor.class, "testMotor");



    public float getDegrees (int beaconIndex, String assetsFileName){
        VuforiaLocalizer.Parameters prams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        prams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        prams.vuforiaLicenseKey = "AaccUNH/////AAABmYn6tLCgBE4CqSKq7IgTGXZ12c4zYDZx4hC67z9E4/R6MU+miAuEK7eHb/uLBUKMWc3BjFb6o60eLN2HX+BwNNH7qG954X5k3pZyYb9MkYQsAaZ/IYv3S6+JSXLDpQFuNXM7HpZCHuxdN1TwFli4SYBLp7//JhheDx1N0xWeowWQMP3WuTQnbA8TnUpaug3H7liINUkllw/wJcGDVnUjlhUdpHODdC2cptlTol8STSYN3oZQvgXcW+xJErdVnqS6Wev3WnIPelnYRZXxL1Ui9qJn49C18cQJdZ1duV26nfzJ2UyuR7tGUKTJzPjDVkbfU3z96goS+bvA08r2sjHgFh1wcdgPKMSsRMcASTf7u7PQ";
        prams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(prams);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("Skystone");


        beacons.get(0).setName("TargetElement");
        beacons.get(1).setName("1");
        beacons.get(2).setName("2");
        beacons.get(3).setName("3");
        beacons.get(4).setName("4");
        beacons.get(5).setName("5");
        beacons.get(6).setName("6");
        beacons.get(7).setName("7");
        beacons.get(8).setName("8");
        beacons.get(9).setName("9");
        beacons.get(10).setName("10");
        beacons.get(11).setName("11");
        beacons.get(12).setName("12");


        beacons.activate();



        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(beaconIndex).getListener()).getPose();

        if (pose != null && pose == beacons.get(beaconIndex)) {
            VectorF translation = pose.getTranslation();

            degreesToTurn = translation.get(0);


        }else{
            degreesToTurn = 0;
        }
        return degreesToTurn;
    }
    public void navToBeacon(int beaconIndex, String assetsFileName){



        VuforiaLocalizer.Parameters prams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        prams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        prams.vuforiaLicenseKey = "AaccUNH/////AAABmYn6tLCgBE4CqSKq7IgTGXZ12c4zYDZx4hC67z9E4/R6MU+miAuEK7eHb/uLBUKMWc3BjFb6o60eLN2HX+BwNNH7qG954X5k3pZyYb9MkYQsAaZ/IYv3S6+JSXLDpQFuNXM7HpZCHuxdN1TwFli4SYBLp7//JhheDx1N0xWeowWQMP3WuTQnbA8TnUpaug3H7liINUkllw/wJcGDVnUjlhUdpHODdC2cptlTol8STSYN3oZQvgXcW+xJErdVnqS6Wev3WnIPelnYRZXxL1Ui9qJn49C18cQJdZ1duV26nfzJ2UyuR7tGUKTJzPjDVkbfU3z96goS+bvA08r2sjHgFh1wcdgPKMSsRMcASTf7u7PQ";
        prams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(prams);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("Skystone");


        beacons.get(0).setName("TargetElement");
        beacons.get(1).setName("1");
        beacons.get(2).setName("2");
        beacons.get(3).setName("3");
        beacons.get(4).setName("4");
        beacons.get(5).setName("5");
        beacons.get(6).setName("6");
        beacons.get(7).setName("7");
        beacons.get(8).setName("8");
        beacons.get(9).setName("9");
        beacons.get(10).setName("10");
        beacons.get(11).setName("11");
        beacons.get(12).setName("12");


        beacons.activate();

    }




}




