package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ServoSynching extends LinearOpMode {

    HardWare robot = new HardWare();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
    }
}
