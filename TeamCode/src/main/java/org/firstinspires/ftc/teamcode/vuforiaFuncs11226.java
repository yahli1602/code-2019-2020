package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name = "Vuforia 11226", group = "Concept")
public class vuforiaFuncs11226 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters prams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        prams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        prams.vuforiaLicenseKey = "AaccUNH/////AAABmYn6tLCgBE4CqSKq7IgTGXZ12c4zYDZx4hC67z9E4/R6MU+miAuEK7eHb/uLBUKMWc3BjFb6o60eLN2HX+BwNNH7qG954X5k3pZyYb9MkYQsAaZ/IYv3S6+JSXLDpQFuNXM7HpZCHuxdN1TwFli4SYBLp7//JhheDx1N0xWeowWQMP3WuTQnbA8TnUpaug3H7liINUkllw/wJcGDVnUjlhUdpHODdC2cptlTol8STSYN3oZQvgXcW+xJErdVnqS6Wev3WnIPelnYRZXxL1Ui9qJn49C18cQJdZ1duV26nfzJ2UyuR7tGUKTJzPjDVkbfU3z96goS+bvA08r2sjHgFh1wcdgPKMSsRMcASTf7u7PQ";
        prams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(prams);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS , 5);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = beacons.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = beacons.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = beacons.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = beacons.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = beacons.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = beacons.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = beacons.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = beacons.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = beacons.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = beacons.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = beacons.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = beacons.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = beacons.get(12);
        rear2.setName("Rear Perimeter 2");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()){
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacons.get(0).getListener()).getPose();
            if(pose != null){
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacons.get(0).getName() + "-Tranlation(0): " , translation.get(0));
                    telemetry.addData(beacons.get(0).getName() + "-Tranlation(1): " , translation.get(1));
                    telemetry.addData(beacons.get(0).getName() + "-Tranlation(2): " , translation.get(2));

                    double degreesToTurn = translation.get(1);

                    telemetry.addData(beacons.get(0).getName() + "-Degrees",degreesToTurn);
                    telemetry.update();
            }





            telemetry.update();
        }

    }
}
