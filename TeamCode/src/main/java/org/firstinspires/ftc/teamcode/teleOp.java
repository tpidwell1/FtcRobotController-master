package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleop")
public class teleOp extends LinearOpMode {

//    private DcMotorEx frontRight   = null;
//    private DcMotorEx         backRight  = null;
//    private DcMotorEx         frontLeft  = null;
//    private DcMotorEx         backLeft  = null;

    private Servo servo = null;

    private NM12351Hardware robot = new NM12351Hardware(this);

    private boolean liftRunning = false, liftSetUp = false;
    private boolean liftRunningb = false, liftSetUpb = false;

    private boolean liftRunningx = false, liftSetUpx = false;
    private boolean servothing = false, servoPosition = false;
    private boolean liftFlip = false, liftFLipPosition = false;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime armflipTime = new ElapsedTime();
    private ElapsedTime armflipbackTime = new ElapsedTime();
    int armflip = -1;
    int armflipback = -1;


    int test = 1;

    private enum LiftPosition {

        HIGH,
        MEDIUM,
        LOW

    }

    private LiftPosition liftPosition = LiftPosition.LOW;


    @Override
    public void runOpMode() throws InterruptedException {


//        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
//        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
//        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
//        backRight = hardwareMap.get(DcMotorEx.class, "br");

        robot.init(hardwareMap);

        robot.rs.setPosition(0);

        robot.cs.setPosition(0.75);

        robot.lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ll.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while (opModeIsActive()) {

//            telemetry.addData("fl power", frontLeft.getPower());
//            telemetry.update();

//            frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
//            backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
//            frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
//            backRight.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);

            robot.fl.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.bl.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.fr.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            robot.br.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);

            telemetry.addData("lift position", robot.lr.getTargetPosition());
            telemetry.addData("left lift enc count", robot.ll.getCurrentPosition());
            telemetry.addData("right lift enc count", robot.lr.getCurrentPosition());
            telemetry.addData("Elapsedtime", armflipTime);

            if (gamepad1.a && !liftRunning) {
                liftRunning = true;
                liftRunningb = false;
                liftRunningx = false;
                robot.lr.setTargetPosition(1600);
                robot.ll.setTargetPosition(-1600);
                robot.lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ll.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if (liftRunning) {
                robot.lr.setPower(1);
                robot.ll.setPower(1);
                liftPosition = LiftPosition.HIGH;
            }

            if ((liftRunning || liftRunningx || liftRunningb) && ((liftPosition == LiftPosition.HIGH || liftPosition == LiftPosition.MEDIUM) && robot.lr.getCurrentPosition() > robot.lr.getTargetPosition() - 20)
                    || (liftPosition == LiftPosition.LOW && robot.lr.getCurrentPosition() < robot.lr.getTargetPosition() + 20)) {
                liftRunning = false;
                robot.stopLiftMotors();
                liftSetUp = false;
            }
            if (gamepad1.x && !liftRunningx) {
                liftRunningx = true;
                liftRunningb = false;
                liftRunning = false;
                robot.lr.setTargetPosition(0);
                robot.ll.setTargetPosition(0);
                robot.lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ll.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
                robot.lr.setPower(0.75);
                robot.ll.setPower(0.75);
                liftSetUp = true;
                liftPosition = LiftPosition.LOW;
            }
            if (gamepad1.b && !liftRunningb) {
                liftRunningb = true;
                liftRunning = false;
                liftRunningx = false;
                robot.lr.setTargetPosition(750);
                robot.ll.setTargetPosition(-750);
                robot.lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.ll.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lr.setPower(1);
                robot.ll.setPower(1);
                liftPosition = LiftPosition.MEDIUM;
            }
/*



            if (gamepad1.a) {
                robot.lr.setPower(1);
                robot.ll.setPower(1);

            } else if (gamepad1.x) {
                robot.lr.setPower.(-1);
                robot.ll.setPower(-1);

            } else {
                robot.ll.setPower(0);
                robot.lr.setPower(0);
            }
*/

            telemetry.addData("armFlipTime", armflipTime.milliseconds());

            if (armflip == -1) {
                armflipTime.reset();

            } else if (armflip == 0) {

                robot.llt.setPower(-1);
                robot.llb.setPower(-1);
                robot.lrt.setPower(1);
                robot.lrb.setPower(1);
            }

                if (armflipTime.milliseconds() > 1500) {

                    telemetry.addData("fjhdsafoieuhdusiafhseoiuagfeiaf", "hifudshafiulehsalfiuehsaliu");
                    telemetry.update();

                    robot.llt.setPower(0);
                    robot.llb.setPower(0);
                    robot.lrt.setPower(0);
                    robot.lrb.setPower(0);

                    armflip = -1;



            }
            if (armflipback == -1) {
                armflipbackTime.reset();

            } else if (armflipback == 0) {
                robot.NMWait(500);
                    robot.llt.setPower(0.35);
                    robot.llb.setPower(0.35);
                    robot.lrt.setPower(-0.35);
                    robot.lrb.setPower(-0.35);

                if (armflipbackTime.milliseconds() > 1100) {
                    robot.llt.setPower(0);
                    robot.llb.setPower(0);
                    robot.lrt.setPower(0);
                    robot.lrb.setPower(0);
                    armflipback = -1;


                }

            }

            if (gamepad1.a || gamepad1.b|| gamepad1.y) {
                robot.rs.setPosition(1);
            }
            if (gamepad1.x) {
                robot.rs.setPosition(0);
            }
                if (gamepad1.a || gamepad1.b ||gamepad1.y) {
                    armflip = 0;

                }
                if (gamepad1.x) {
                    armflipback = 0;
                }
                if (gamepad1.dpad_up){
                    robot.cs.setPosition(0.75);
                }
                if (gamepad1.dpad_down){
                   robot.cs.setPosition(0.2);
                }


                //if(armflipTime.milliseconds() > ~){

            //telemetry.addData("servo position", robot.llt.getPosition());
//            telemetry.update();
        }
    }
}



