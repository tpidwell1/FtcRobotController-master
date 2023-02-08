package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NM12351Hardware;

@Autonomous(name = "lifttest", group = "Group 1")
public class lifttest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private NM12351Hardware robot = new NM12351Hardware(this);

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        waitForStart();

        robot.liftPositionHigh(50  , 0.5);
    }
}





