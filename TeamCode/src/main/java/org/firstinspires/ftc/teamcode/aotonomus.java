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

@TeleOp(name = "aotonomus 11226", group = "Concept")
public class aotonomus {

    public void degreesToTurn () throws InterruptedException {
        VuforiaLocalizer.Parameters prams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        prams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        prams.vuforiaLicenseKey = "AaccUNH/////AAABmYn6tLCgBE4CqSKq7IgTGXZ12c4zYDZx4hC67z9E4/R6MU+miAuEK7eHb/uLBUKMWc3BjFb6o60eLN2HX+BwNNH7qG954X5k3pZyYb9MkYQsAaZ/IYv3S6+JSXLDpQFuNXM7HpZCHuxdN1TwFli4SYBLp7//JhheDx1N0xWeowWQMP3WuTQnbA8TnUpaug3H7liINUkllw/wJcGDVnUjlhUdpHODdC2cptlTol8STSYN3oZQvgXcW+xJErdVnqS6Wev3WnIPelnYRZXxL1Ui9qJn49C18cQJdZ1duV26nfzJ2UyuR7tGUKTJzPjDVkbfU3z96goS+bvA08r2sjHgFh1wcdgPKMSsRMcASTf7u7PQ";
        prams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(prams);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS , 4);

        VuforiaTrackables targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");



        targetsSkyStone.activate();

        while(opModeIsActive()){
            for (VuforiaTrackable beac : targetsSkyStone){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();



                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));


                }
            }

        }

        return 0;
    }
}
