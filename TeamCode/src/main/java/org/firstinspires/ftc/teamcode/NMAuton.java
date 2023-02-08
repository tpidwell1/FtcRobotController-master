package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Aton 1", group = "Group 1")
public class NMAuton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private NM12351Hardware robot = new NM12351Hardware(this);

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        waitForStart();

        robot.driveForCounts(1550, -0.5);
        robot.NMWait(100);
        robot.strafeForCounts(325,0.5);
        robot.NMWait(200);
        robot.strafeForCounts(325,-0.5);
        robot.turnForCounts(650,-0.5);
        robot.driveForCounts(750,0.5);
        robot.NMWait(200);
        robot.driveForCounts(750,-0.5);
        robot.turnForCounts(650,0.5);
        robot.strafeForCounts(325,0.5);
        robot.NMWait(200);
        robot.strafeForCounts(325,-0.5);

        //robot.turnForCounts(720,0.5);
        //robot.driveForCounts(750,0.5);

        for (int i = 0; i < 5; i++) {

          /*   robot.NMWait(100);
             robot.driveForCounts(700,-0.5);
             robot.NMWait(100);
             robot.turnForCounts(740,-0.5);
             //robot.NMWait(500);
             //robot.strafeForCounts(250,-0.5);
             //robot.NMWait(500);
             //robot.strafeForCounts(250,0.5);
             robot.NMWait(100);
             robot.turnForCounts(660,0.5);
             robot.NMWait(100);
             robot.driveForCounts(740,0.5);
            // robot.NMWait(100);
            /*
             



        }

        /*robot.driveForCounts(1700,0.5);
        robot.NMWait(100);
        //robot.strafeForCounts(1000,-0.5);
        robot.NMWait(100);
        robot.turnForCounts(700,0.5);
        robot.driveForCounts(750,0.5);
        robot.NMWait(500);
        robot.driveForCounts(75
       0,-0.5);
        robot.NMWait(100);
        robot.turnForCounts(700,-0.5);
        robot.NMWait(500);
        robot.turnForCounts(700,0.5);
        robot.NMWait(100);
        robot.driveForCounts(1000,0.5);
        robot.NMWait(500);
        robot.driveForCounts(1000,-0.5);
        robot.NMWait(100);
        robot.turnForCounts(700,-0.5);
        robot.NMWait(500);
        robot.turnForCounts(700,0.5);
        robot.NMWait(100);
        */
        }
    }
}


