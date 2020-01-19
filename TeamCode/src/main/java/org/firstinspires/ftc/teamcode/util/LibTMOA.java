/*
 *   Copyright (c) 2019 under GPL v3 license. All rights reserved.
 *   Created by Manuel Díaz and Obed García with help of Paolo Reyes for WinT 15645 Subteam of WinT 3794
 *   This code was used in FIRST Tech Challenge 2019 - 2020
 *   Trigonometric Mecanum Omnidriving Algorithm
 */
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class LibTMOA {
  private DcMotor SP_IZQ;
  private DcMotor IN_IZQ;
  private DcMotor SP_DRC;
  private DcMotor IN_DRC;

  Double width;
  Integer cpr;
  Integer gearratio;
  Double diameter;
  Double cpi;

  private final Double bias = 0.91;
  private final Double meccyBias = 0.9;

  Double conversion;
  Boolean exit = false;

  public LibTMOA(
    DcMotor SP_IZQ,
    DcMotor IN_IZQ,
    DcMotor SP_DRC,
    DcMotor IN_DRC,
    double width,
    int cpr,
    int gearratio,
    double diameter
  ) {
    this.SP_IZQ = SP_IZQ;
    this.SP_DRC = SP_DRC;
    this.IN_IZQ = IN_IZQ;
    this.IN_DRC = IN_DRC;

    this.width = width;
    this.cpr = cpr;
    this.gearratio = gearratio;
    this.diameter = diameter;

    this.cpi = (cpr * gearratio) / (Math.PI * diameter);
    this.conversion = cpi * bias;
  }

  public LibTMOA(
    DcMotor SP_IZQ,
    DcMotor IN_IZQ,
    DcMotor SP_DRC,
    DcMotor IN_DRC
  ) {
    this.SP_IZQ = SP_IZQ;
    this.SP_DRC = SP_DRC;
    this.IN_IZQ = IN_IZQ;
    this.IN_DRC = IN_DRC;

    this.width = 16.16;
    this.cpr = 28;
    this.gearratio = 20;
    this.diameter = 2.952755906;
    this.cpi = (cpr * gearratio) / (Math.PI * diameter);
  }

  public void rotate(double speed) {
    this.SP_IZQ.setPower(-1 * speed);
    this.IN_IZQ.setPower(-1 * speed);
    this.SP_DRC.setPower(speed);
    this.IN_DRC.setPower(speed);
  }

  public void rotateIn(double speed) {
    this.SP_IZQ.setPower(speed);
    this.SP_DRC.setPower(-speed);
    this.IN_IZQ.setPower(0);
    this.IN_DRC.setPower(0);
  }

  public void rotate(double speed, double angle) {
    this.SP_IZQ.setPower(speed * (angle / (Math.PI / 4)));
    this.IN_DRC.setPower(-speed * (angle / (Math.PI / 4)));
    this.SP_DRC.setPower(-speed * (angle / (Math.PI / 4)));
    this.IN_IZQ.setPower(speed * (angle / (Math.PI / 4)));
  }

  public void usingBrake() {
    this.SP_IZQ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.IN_IZQ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.SP_DRC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.IN_DRC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void withoutBrake() {
    this.SP_IZQ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    this.IN_IZQ.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    this.SP_DRC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    this.IN_DRC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }

  public static double calc1(double Vd, double Td, double Vt) {
    double V;

    V = Vd * Math.sin(Td + (Math.PI / 4)) + Vt;
    return V;
  }

  public static double calc2(double Vd, double Td, double Vt) {
    double V;
    V = Vd * Math.cos(Td + (Math.PI / 4)) + Vt;
    return V;
  }

  public void move(double Vd, double Td, double Vt) {
    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;

    v1 = this.calc1(Vd, Td, Vt);
    v2 = this.calc2(Vd, Td, Vt);
    v3 = this.calc2(Vd, Td, Vt);
    v4 = this.calc1(Vd, Td, Vt);

    this.SP_DRC.setPower(v1);
    this.SP_IZQ.setPower(v2);
    this.IN_DRC.setPower(v3);
    this.IN_IZQ.setPower(v4);
  }

  public void move(double x, double y) {
    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;

    double Vd = 0;
    double Td = 0;

    if (x + y <= 1 || x + y >= -1) {
      Vd = this.speed(x, y);
      Td = this.angle(x, y);

      v1 = this.calc1(Vd, Td, 0);
      v2 = this.calc2(Vd, Td, 0);
      v3 = this.calc2(Vd, Td, 0);
      v4 = this.calc1(Vd, Td, 0);

      this.SP_DRC.setPower(v1);
      this.SP_IZQ.setPower(v2);
      this.IN_DRC.setPower(v3);
      this.IN_IZQ.setPower(v4);
    }
  }

  public void move(double x, double y, double Vd, double Vt) {
    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;

    double Td = 0;

    if (x + y <= 1 || x + y >= -1) {
      Td = this.angle(x, y);

      v1 = this.calc1(Vd, Td, Vt);
      v2 = this.calc2(Vd, Td, Vt);
      v3 = this.calc2(Vd, Td, Vt);
      v4 = this.calc1(Vd, Td, Vt);

      this.SP_DRC.setPower(v1);
      this.SP_IZQ.setPower(v2);
      this.IN_DRC.setPower(v3);
      this.IN_IZQ.setPower(v4);
    }
  }

  public double angle(double x, double y) {
    double a = Math.atan2(x, y);
    return a;
  }

  public double speed(double x, double y) {
    double s = 0;
    s = x + y;
    return s;
  }

  public void stop() {
    this.SP_DRC.setPower(0);
    this.SP_IZQ.setPower(0);
    this.IN_DRC.setPower(0);
    this.IN_IZQ.setPower(0);
  }

  public void withoutEncoders() {
    SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IN_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    IN_IZQ.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }
  
  public void usingEncoders() {
    SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IN_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    IN_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void targetToPositionEncoders() {
    IN_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_IZQ.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    SP_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    SP_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    IN_IZQ.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    IN_DRC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  public void moveToPosition(double inches, double speed) {
    int move = (int) (Math.round(inches * conversion));

    IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
    SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() + move);
    IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() + move);
    SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

    SP_IZQ.setPower(speed);
    IN_IZQ.setPower(speed);
    SP_DRC.setPower(speed);
    IN_DRC.setPower(speed);

    while (
      SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
    ) {
      if (exit) {
        SP_DRC.setPower(0);
        SP_IZQ.setPower(0);
        IN_DRC.setPower(0);
        IN_IZQ.setPower(0);
        return;
      }
    }
    SP_DRC.setPower(0);
    SP_IZQ.setPower(0);
    IN_DRC.setPower(0);
    IN_IZQ.setPower(0);
    return;
  }

  public void mecanumToPosition(double inches, double speed) {
    
    targetToPositionEncoders();
    int move = (int) (Math.round(inches * conversion));

    IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
    SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() - move);
    IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
    SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() + move);

    SP_IZQ.setPower(calc2(speed, 0, 0));
    IN_IZQ.setPower(calc1(speed, 0, 0));
    SP_DRC.setPower(calc1(speed, 0, 0));
    IN_DRC.setPower(calc2(speed, 0, 0));

    /*while (
      SP_IZQ.isBusy() && SP_DRC.isBusy() && IN_IZQ.isBusy() && IN_DRC.isBusy()
    ) {
      if (exit) {
        SP_DRC.setPower(0);
        SP_IZQ.setPower(0);
        IN_DRC.setPower(0);
        IN_IZQ.setPower(0);
        return;
      }
    }*/
    return;
  }

  public void turnWithEncoder(double input) {
    SP_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    IN_IZQ.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    SP_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    IN_DRC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //
    SP_IZQ.setPower(input);
    IN_IZQ.setPower(input);
    SP_DRC.setPower(-input);
    IN_DRC.setPower(-input);
  }

  public void turnToPosition(double inches, double speed) {
    targetToPositionEncoders();
    int move = (int) (Math.round(inches * conversion));

    IN_IZQ.setTargetPosition(IN_IZQ.getCurrentPosition() + move);
    IN_DRC.setTargetPosition(IN_DRC.getCurrentPosition() - move);
    SP_DRC.setTargetPosition(SP_DRC.getCurrentPosition() - move);
    SP_IZQ.setTargetPosition(SP_IZQ.getCurrentPosition() + move);

    SP_IZQ.setPower(speed);
    IN_IZQ.setPower(speed);
    SP_DRC.setPower(speed);
    IN_DRC.setPower(speed);
  }
  
  public void moveWithoutLimits(double Td){
    double[] v = {0,0,0,0};

      v[0] = Math.round(this.calc1(1, Td, 0));
      v[1] = Math.round(this.calc2(1, Td, 0));
      v[2] = Math.round(this.calc2(1, Td, 0));
      v[3] = Math.round(this.calc1(1, Td, 0));
      
      this.SP_DRC.setPower(v[0]);
      this.SP_IZQ.setPower(v[1]);
      this.IN_DRC.setPower(v[2]);
      this.IN_IZQ.setPower(v[3]);
    
  }
}
