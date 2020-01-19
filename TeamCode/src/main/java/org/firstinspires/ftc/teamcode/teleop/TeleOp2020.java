/*
 *   Copyright (c) 2019 under MIT license. All rights reserved.
 *   Created by Manuel Díaz and Obed García with help Paolo Reyes for WinT 15645 Subteam of WinT 3794
 *   This code was used in FIRST Tech Challenge 2019 - 2020
 *   Joker Robot TeleOp Program
 */
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.LibTMOA;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "Joker")
public class TeleOp2020 extends LinearOpMode {
  Double width = 16.16; //inches
  Integer cpr = 28; //counts per rotation
  Integer gearratio = 20;
  Double diameter = 2.952755906;
  Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
  Double bias = 0.91; //default 0.8
  Double meccyBias = 0.9;
  Double conversion = cpi * bias;
  Boolean exit = false;

  private DcMotor SP_DRC;
  private DcMotor IN_DRC;
  private DcMotor SP_IZQ;
  private Servo JL_DRC;
  private Servo JL_IZQ;
  private DcMotor IN_IZQ;
  private DcMotor ELE_P;
  private DcMotor BRZ;
  private DcMotor ELE_A1;
  private DcMotor ELE_A2;
  private Servo CUBO;
  private Servo RLQ;

  double Vd = 0;
  double Td = 0;
  double Vt = 0;

  double jsX = 0;
  double jsY = 0;
  double jsY2 = 0;
  double jsRY = 0;

  private LibTMOA mecanum;

  @Override
  public void runOpMode() {
    SP_DRC = hardwareMap.dcMotor.get("SP_DRC");
    IN_DRC = hardwareMap.dcMotor.get("IN_DRC");
    SP_IZQ = hardwareMap.dcMotor.get("SP_IZQ");
    JL_DRC = hardwareMap.servo.get("JL_DRC");
    JL_IZQ = hardwareMap.servo.get("JL_IZQ");
    IN_IZQ = hardwareMap.dcMotor.get("IN_IZQ");
    BRZ = hardwareMap.dcMotor.get("BRZ");
    ELE_P = hardwareMap.dcMotor.get("ELE_P");
    ELE_A1 = hardwareMap.dcMotor.get("ELE_A1");
    ELE_A2 = hardwareMap.dcMotor.get("ELE_A2");
    CUBO = hardwareMap.servo.get("CUBO");
    RLQ = hardwareMap.servo.get("RLQ");

    SP_IZQ.setDirection(DcMotorSimple.Direction.REVERSE);
    IN_IZQ.setDirection(DcMotorSimple.Direction.REVERSE);
    ELE_A1.setDirection(DcMotorSimple.Direction.REVERSE);
    ELE_A2.setDirection(DcMotorSimple.Direction.REVERSE);
    BRZ.setDirection(DcMotorSimple.Direction.REVERSE);

    SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ELE_P.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ELE_A1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ELE_A2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ELE_P.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ELE_A1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ELE_A2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    JL_DRC.setPosition(0);
    JL_IZQ.setPosition(1);

    mecanum = new LibTMOA(SP_IZQ, IN_IZQ, SP_DRC, IN_DRC);
    waitForStart();

    if (opModeIsActive()) {
      while (opModeIsActive()) {
        Vd = 0;
        Td = 0;
        Vt = 0;

        jsY = -gamepad1.left_stick_y;
        jsX = gamepad1.right_stick_x;
        jsRY = -gamepad1.right_stick_y;
        jsY2 = -gamepad2.right_stick_y;

        if (
          jsX == 0 &&
          jsY == 0 &&
          gamepad1.left_trigger <= 0 &&
          gamepad1.right_trigger <= 0 &&
          !gamepad1.left_bumper &&
          !gamepad1.right_bumper
        ) {
          mecanum.usingBrake();
          Vd = 0;
        } else {
          mecanum.withoutBrake();
        }

        climbElevator(gamepad2.left_stick_y);
        if (gamepad2.y) {
          jsY2 = -0.314;
        }
        climbArm(jsY2);

        if (
          gamepad1.dpad_up ||
          gamepad1.dpad_right ||
          gamepad1.dpad_down ||
          gamepad1.dpad_left
        ) {
          Vd = 0.5;
          if (gamepad1.dpad_up) {
            if (gamepad1.dpad_right) {
              Td = 3 * Math.PI / 4;
            }
            if (gamepad1.dpad_left) {
              Td = Math.PI / 4;
            } else {
              Td = 0;
            }
          } else if (gamepad1.dpad_down) {
            if (gamepad1.dpad_right) {
              Td = 5 * Math.PI / 4;
            }
            if (gamepad1.dpad_left) {
              Td = 7 * Math.PI / 4;
            } else {
              Td = Math.PI;
            }
          } else if (gamepad1.dpad_right) {
            Td = 3 * Math.PI / 2;
          } else if (gamepad1.dpad_left) {
            Td = Math.PI / 2;
          }
        }

        if (gamepad2.right_bumper) {
          CUBO.setPosition(0);
        } else {
          CUBO.setPosition(1);
        }
        if (gamepad2.left_bumper) {
          RLQ.setPosition(1);
        } else {
          RLQ.setPosition(0);
        }

        if (gamepad1.a) {
          JL_DRC.setPosition(1);
          JL_IZQ.setPosition(0);
        } else {
          JL_DRC.setPosition(0);
          JL_IZQ.setPosition(1);
        }

        if (gamepad1.right_bumper) {
          mecanum.usingBrake();
          Vd = 1;
          Td = 3 * Math.PI / 2;
        } else if (gamepad1.left_bumper) {
          mecanum.usingBrake();
          Vd = 1;
          Td = Math.PI / 2;
        }

        if (jsY != 0) {
          Vd = 0;
          Td = 0;
          Vt = jsY;
        }

        if (jsRY + jsX != 0) {
          Vd = 0.8;
          Td = mecanum.angle(-jsX, jsRY);
          Vt = 0;
        }

        mecanum.move(Vd, Td, Vt);

        if (gamepad1.right_trigger > 0) {
          mecanum.rotate(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
          mecanum.rotate(gamepad1.left_trigger);
        }

        if (gamepad1.b) {
          JL_IZQ.setPosition(0);
          JL_DRC.setPosition(1);
          sleep(500);
          mecanum.move(0.6, 0, 0);
          sleep(2000);
          mecanum.stop();
        } else if (gamepad1.a) {
          JL_IZQ.setPosition(0);
          JL_DRC.setPosition(1);
        } else {
          JL_IZQ.setPosition(1);
          JL_DRC.setPosition(0);
        }


        telemetry.addLine("Vd: " + Vd);
        telemetry.addLine("Td: " + Td);
        telemetry.addLine("jsX:" + jsX);
        telemetry.addLine("jsY:" + jsY);
        telemetry.addLine("jsY2: " + jsY2);
        telemetry.update();
      }
    }
  }

  private void climbElevator(double speed) {
    ELE_P.setPower(speed);
    ELE_A1.setPower(speed);
    ELE_A2.setPower(speed);
  }

  private void climbArm(double speed) {
    BRZ.setPower(speed);
  }

  private void climbElevatorToPosition(double inches, double speed) {
    int move = (int) (Math.round(inches * conversion));

    ELE_P.setTargetPosition(ELE_P.getCurrentPosition() + move);
    ELE_A1.setTargetPosition(ELE_A1.getCurrentPosition() + move);
    ELE_A2.setTargetPosition(ELE_A2.getCurrentPosition() + move);

    ELE_P.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ELE_A1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ELE_A2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    ELE_P.setPower(speed);
    ELE_A1.setPower(speed);
    ELE_A2.setPower(speed);

    while (ELE_A1.isBusy() && ELE_P.isBusy() && ELE_A2.isBusy()) {
      if (exit) {
        ELE_P.setPower(0);
        ELE_A1.setPower(0);
        ELE_A2.setPower(0);
        return;
      }
    }
    ELE_P.setPower(0);
    ELE_A1.setPower(0);
    ELE_A2.setPower(0);
    return;
  }

  public void withoutEncoders() {
    ELE_P.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ELE_A1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ELE_A2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ELE_P.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ELE_A1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    ELE_A2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }
}
