
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Teleop_11229", group="Linear Opmode")

public class Teleop_11229 extends LinearOpMode {
private Funcs_11229 funcs = new Funcs_11229();

    @Override
    public void runOpMode() {

        waitForStart();


        while (opModeIsActive()) {
            funcs.drive();
            funcs.fourBar();
            funcs.collect();
        }
    }
}
