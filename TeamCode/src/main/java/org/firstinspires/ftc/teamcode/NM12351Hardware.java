/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class NM12351Hardware {

    /* Declare OpMode members. */
    LinearOpMode op;
    HardwareMap hwMap;
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx bl = null;
    public DcMotorEx br = null;
    public DcMotorEx ll = null;
    public DcMotorEx lr = null;
   // public DcMotorEx lift1 = null;
   // public DcMotorEx lift2 = null;

    public CRServo llt = null;
    public CRServo llb = null;
    public CRServo lrt = null;
    public CRServo lrb = null;
    public Servo rs = null;
    public Servo cs = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    private boolean liftFirstTimeRunning = true;
    int readCounts = 0;
    private ElapsedTime armflipTime = new ElapsedTime();
    private ElapsedTime armflipbackTime = new ElapsedTime();
    int armflip = -1;
    int armflipback = -1;

    // Define a constructor that allows the OpMode to pass a reference to itself.
//    public NM12351Hardware(OpMode opmode) {
//        myOpMode = opmode;
//    }

    public NM12351Hardware(LinearOpMode opmode) {
        this.op = opmode;
    }

    /**
     * Initialize all the robot's hardware.8
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hwMap) {

        this.hwMap = hwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        fr = hwMap.get(DcMotorEx.class, "fr");
        fl = hwMap.get(DcMotorEx.class, "fl");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");
        ll = hwMap.get(DcMotorEx.class, "ll");
        lr = hwMap.get(DcMotorEx.class, "lr");
        llt =hwMap.get(CRServo.class,"llt");
        llb =hwMap.get(CRServo.class,"llb");
        lrt =hwMap.get(CRServo.class,"lrt");
        lrb =hwMap.get(CRServo.class,"lrb");
        rs =hwMap.get(Servo.class,"rs");
        cs =hwMap.get(Servo.class,"cs");
       // lift1 = hwMap.get(DcMotorEx.class, "lift");
       // lift2 = hwMap.get(DcMotorEx.class, "lift2");
        //armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ll.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
       // os = hwMap.get(Servo.class, "os");
        // rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
       // os.setPosition(0);
        //rightHand.setPosition(MID_SERVO);


        op.telemetry.addData(">", "Hardware Initialized");
        op.telemetry.update();
    }

    public void driveForCounts(int counts, double power) {
        int readCounts = 0;

        resetAllEncoders();

        runWithoutEncoders();

        op.telemetry.addData("driveForCounts", 1);
        op.telemetry.addData("Counts", counts);
        op.telemetry.update();
        setDriveMotors(power);
        while (Math.abs(readCounts) < counts) {
            readCounts = fr.getCurrentPosition();
            readCounts = fl.getCurrentPosition();
            readCounts = bl.getCurrentPosition();
            readCounts = br.getCurrentPosition();
            op.telemetry.addData("readCounts", readCounts);
            op.telemetry.update();

        }
        stopDriveMotors();
    }

    public void setDriveMotors(double
                                       power) {
        op.telemetry.addData("setDriveMotors", power);
        op.telemetry.update();

        fr.setPower(power);
        fl.setPower(power);
        br.setPower(power);
        bl.setPower(power);

    }

    public void stopDriveMotors() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }
    public void stopLiftMotors() {
        lr.setPower(0);
        ll.setPower(0);
    }

    public void BRAKE() {
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void NMWait(int howLong) {
        runtime.reset();
        while (runtime.milliseconds() < howLong && op.opModeIsActive() && !op.isStopRequested()) {
        }
    }

    public void resetAllEncoders() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders() {
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ll.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnForCounts(int counts, double power) {
        int readCounts = 0;

        resetAllEncoders();

        runWithoutEncoders();

        op.telemetry.addData("driveForCounts", 1);
        op.telemetry.addData("Counts", counts);
        op.telemetry.update();
        setTurnPower(power);
        while (Math.abs(readCounts) < counts) {
            readCounts = fr.getCurrentPosition();
            op.telemetry.addData("readCounts", readCounts);
            op.telemetry.update();
        }
        stopDriveMotors();
    }
    public void setTurnPower(double power) {
        op.telemetry.addData("setDriveMotors", power);
        op.telemetry.update();

        fr.setPower(power);
        fl.setPower(-power);
        br.setPower(power);
        bl.setPower(-power);

    }
    //public void setOs(double position){
        //os.setPosition(position);
   // }
    public void strafeForCounts(int counts, double power) {
        int readCounts = 0;

        resetAllEncoders();

        runWithoutEncoders();

        op.telemetry.addData("driveForCounts", 1);
        op.telemetry.addData("Counts", counts);
        op.telemetry.update();
        setStrafePower(power);
        while (Math.abs(readCounts) < counts) {
            readCounts = fr.getCurrentPosition();
            op.telemetry.addData("readCounts", readCounts);
            op.telemetry.update();
        }
        stopDriveMotors();
    }
    public void setStrafePower(double power) {
        op.telemetry.addData("setDriveMotors", power);
        op.telemetry.update();

        fr.setPower(power);
        fl.setPower(-power);
        br.setPower(-power);
        bl.setPower(power);

    }
    public void liftPositionHigh(int counts, double power) {

        op.telemetry.addData("liftPositionHigh", 1);
        op.telemetry.addData("Counts", counts);
        op.telemetry.update();

        lr.setTargetPosition(counts);
        ll.setTargetPosition(counts);

        while (liftFirstTimeRunning && op.gamepad1.a && op.gamepad1.x) {

            lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ll.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        liftFirstTimeRunning = false;
        setLiftPositionHigh(power);


        //        while (Math.abs(readCounts) < counts) {
//            readCounts = lr.getCurrentPosition();
//            op.telemetry.addData("readCounts", readCounts);
//            op.telemetry.update();
//        }
//        stopLiftMotors();
        if (counts == 0 && lr.getCurrentPosition() < lr.getTargetPosition() + 10) {
            stopLiftMotors();
        } else if (lr.getCurrentPosition() > lr.getTargetPosition() - 10 || lr.getCurrentPosition() < lr.getTargetPosition() + 10 ) {
            setLiftPositionHigh(0);
        }
    }
    public void setLiftPositionHigh(double power) {
        op.telemetry.addData("liftMotors", power);
        op.telemetry.update();

        lr.setPower(power);
        ll.setPower(power);
    }

    public void liftForCounts(int counts, double power) {
        int readCounts = 0;

        resetAllEncoders();

        runWithoutEncoders();

        op.telemetry.addData("driveForCounts", 1);
        op.telemetry.addData("Counts", counts);
        op.telemetry.update();
        setLiftPower(power);
        while (Math.abs(readCounts) < counts) {
            readCounts = lr.getCurrentPosition();
            op.telemetry.addData("readCounts", readCounts);
            op.telemetry.update();
        }
    }
    public void setLiftPower(double power) {
        op.telemetry.addData("setDriveMotors", power);
        op.telemetry.update();

        lr.setPower(power);
        ll.setPower(-power);

        }
    public void liftDownForCounts(int counts, double power) {
        int readCounts = 0;

        resetAllEncoders();

        runWithoutEncoders();

        op.telemetry.addData("driveForCounts", 1);
        op.telemetry.addData("Counts", counts);
        op.telemetry.update();
        setDownLiftPower(power);
        while (Math.abs(readCounts) < counts) {
            readCounts = lr.getCurrentPosition();
            op.telemetry.addData("readCounts", readCounts);
            op.telemetry.update();
        }
        stopLiftMotors();
    }
    public void setDownLiftPower(double power) {
        op.telemetry.addData("setDriveMotors", power);
        op.telemetry.update();

        lr.setPower(-power);
        ll.setPower(power);

    }
    public void flipLift() {
        if (armflip == -1) {
            armflipTime.reset();
        }
        while (armflipTime.milliseconds() < 1350) {
            llt.setPower(-0.75);
            llb.setPower(-0.75);
            lrt.setPower(0.75);
            lrb.setPower(0.75);

            armflip = -1;
        }
    }
        public void flipLiftBack() {
            if (armflipback == -1) {
                armflipbackTime.reset();
            }
            while (armflipbackTime.milliseconds() < 500) {
                llt.setPower(0.35);
                llb.setPower(0.35);
                lrt.setPower(-0.35);
                lrb.setPower(-0.35);

                armflipback = -1;
            }

    }

    }










