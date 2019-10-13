package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class vuforiaOp11226 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters prams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        prams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        prams.vuforiaLicenseKey = "AaccUNH/////AAABmYn6tLCgBE4CqSKq7IgTGXZ12c4zYDZx4hC67z9E4/R6MU+miAuEK7eHb/uLBUKMWc3BjFb6o60eLN2HX+BwNNH7qG954X5k3pZyYb9MkYQsAaZ/IYv3S6+JSXLDpQFuNXM7HpZCHuxdN1TwFli4SYBLp7//JhheDx1N0xWeowWQMP3WuTQnbA8TnUpaug3H7liINUkllw/wJcGDVnUjlhUdpHODdC2cptlTol8STSYN3oZQvgXcW+xJErdVnqS6Wev3WnIPelnYRZXxL1Ui9qJn49C18cQJdZ1duV26nfzJ2UyuR7tGUKTJzPjDVkbfU3z96goS+bvA08r2sjHgFh1wcdgPKMSsRMcASTf7u7PQ";
        prams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(prams);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS , 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Geers");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()){
            for (VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Tranlation" , translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees",degreesToTurn);
                }
            }
            telemetry.update();
        }

    }
}
