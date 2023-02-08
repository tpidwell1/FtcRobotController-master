package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Aton 2", group = "Group 1")
public class LeftSideAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private NM12351Hardware robot = new NM12351Hardware(this);

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        waitForStart();

        robot.flipLiftBack();

    }
}
