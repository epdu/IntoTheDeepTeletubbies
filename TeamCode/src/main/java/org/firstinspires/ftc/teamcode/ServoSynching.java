package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ServoSynching extends LinearOpMode {

    HardWare robot = new HardWare();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.zero.setPosition(0);
        robot.one.setPosition(1);
    }
}
