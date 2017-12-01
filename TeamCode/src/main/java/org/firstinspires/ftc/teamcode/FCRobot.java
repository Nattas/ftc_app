/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "FullControl", group = "TeleOp")
public class FCRobot extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor arm = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private Servo julinator = null;
    private BNO055IMU gyro = null;
    private int armInitial = 0;
    private double armFinal = 0;
    private int MULTI = 1;
    private int LINEBYLINE = 2;
    private int RIGHT = 1;
    private int LEFT = 2;
    private int MODE = MULTI;
    private int motorTickPerRevolution = (1440 / 86) * 100;
    private double wheelDiameter = 2 * 5.1 * 3.14;
    private double distanceBetween = 39;
    private double maxArmCentimeter = 20;
    private double placeSpinDiameter = 2 * distanceBetween * 3.14;
    private double placeOutSpinDiameter = distanceBetween * 3.14;
    private double tickPerCentimeter = (motorTickPerRevolution) / (wheelDiameter);
    private double armToGear = 3 / 6.5;
    private double armLength = 28.8;
    private double armCmPerMotorRevolution = (2 * armLength * 3.14) * armToGear;
    private double tickPerArmCentimeter = (motorTickPerRevolution / armCmPerMotorRevolution);
    private double tickPerDegree = ((placeSpinDiameter * tickPerCentimeter / 360) / 231) * 245;
    private double tickPerDegreeAtPlace = ((placeOutSpinDiameter * tickPerCentimeter / 360) / 231) * 245;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<String> permanent_messages = new ArrayList<>();

    @Override
    public void init() {
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        initializeMotors();
        initializeServos();
        collapseClaw();
        collapseJulinator();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        //        callibrateGyro();
        clawOpen();
    }

    @Override
    public void loop() {
        checkEmergency();
        handleGamepad1();
        handleGamepad2();
        handleActions();
        showMessages();
    }

    @Override
    public void stop() {
        collapseClaw();
        resetRobot();
    }

    Action[] getAutonomous() {
        Action[] autonomous = new Action[]{
                autonomousDrive(20),
                autonomousClawOpen(),
                autonomousDrive(20),
                autonomousClawClose(),
                autonomousArmMove(10),
                autonomousTurnLeft(90),
                autonomousDrive(30),
                autonomousArmMove(-9),
                autonomousClawOpen(),
                autonomousDrive(-20),
                new Action(new Action.Execute() {
                    @Override
                    public void onSetup() {
                        MODE = MULTI;
                    }

                    @Override
                    public boolean onLoop() {
                        return true;
                    }
                })};
        return autonomous;
    }

    double getDrivePower() {
        double power = 0;
        if (gamepad1.right_trigger != 0) {
            power += gamepad1.right_trigger;
        }
        if (gamepad1.left_trigger != 0) {
            power -= gamepad1.left_trigger / 3;
        }
        return power;
    }

    double getTurnPower() {
        double power = 0;
        if (gamepad1.dpad_left) {
            power -= 1;
        } else if (gamepad1.dpad_right) {
            power += 1;
        } else {
            power += 0;
        }
        return power / 2;
    }

    double getArmSpeed() {
        double speed = 0;
        if (gamepad2.right_trigger != 0) {
            speed += gamepad2.right_trigger;
        }
        if (gamepad2.left_trigger != 0) {
            speed -= gamepad2.left_trigger / 4;
        }
        return speed * 0.7;
    }

    boolean isClawOpen() {
        return clawLeft.getPosition() < 0.5 && clawRight.getPosition() < 0.5;
    }

    boolean isArmUp() {
        return arm.getCurrentPosition() > armInitial;
    }

    boolean isJulinatorDown() {
        return julinator.getPosition() > 0.05;
    }

    Action autonomousTurnRightAtPlace(final int degree) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerDegreeAtPlace * degree));
                leftDrive.setPower(1);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - (int) (tickPerDegreeAtPlace * degree));
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousTurnLeftAtPlace(final int degree) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegreeAtPlace * degree));
                rightDrive.setPower(1);
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - (int) (tickPerDegreeAtPlace * degree));
                leftDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousTurnRight(final int degree) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                leftDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousTurnLeft(final int degree) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousArmMove(final int cm) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(arm.getCurrentPosition() + (int) (tickPerArmCentimeter * -cm));
                arm.setPower(0.7);
            }

            @Override
            public boolean onLoop() {
                if (!arm.isBusy()) {
                    resetArm();
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousJulinatorUp() {
        Action a = new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julinator.setPosition(0.05);
            }

            @Override
            public boolean onLoop() {
                return !isJulinatorDown();
            }
        });
        return a;
    }

    Action autonomousJulinatorDown() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julinator.setPosition(0.45);
            }

            @Override
            public boolean onLoop() {
                return isJulinatorDown();
            }
        });
    }

    Action autonomousClawOpen() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                clawLeft.setPosition(0.4);
                clawRight.setPosition(0.4);
            }

            @Override
            public boolean onLoop() {
                return isClawOpen();
            }
        });
    }

    Action autonomousClawClose() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.5);
            }

            @Override
            public boolean onLoop() {
                return !isClawOpen();
            }
        });
    }

    Action autonomousDrive(final int centimeters) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    @Deprecated
    Action autonomousDriveToGyro(final float angle) {
        return new Action(new Action.Execute() {
            int startPosition;

            @Override
            public void onSetup() {
                startPosition = leftDrive.getCurrentPosition();
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousDone() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                MODE = MULTI;
            }

            @Override
            public boolean onLoop() {
                return true;
            }
        });
    }

    void autoTurnLeft(final int degree) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }

    void autoTurnRight(final int degree) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                leftDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }

    void autoReorientForCube(int direction) {
        MODE = LINEBYLINE;
        actions.clear();
        if (direction == RIGHT) {
            actions.add(autonomousDrive(-20));
            actions.add(autonomousTurnRightAtPlace(85));
            actions.add(autonomousTurnLeft(45));
            actions.add(autonomousTurnLeftAtPlace(30));
            actions.add(autonomousDone());
        } else if (direction == LEFT) {
            actions.add(autonomousDrive(-20));
            actions.add(autonomousTurnLeftAtPlace(85));
            actions.add(autonomousTurnRight(45));
            actions.add(autonomousTurnRightAtPlace(30));
            actions.add(autonomousDone());
        }
    }

    void handleActions() {
        if (MODE == LINEBYLINE) {
            if (actions.size() != 0) {
                if (actions.get(0).isAlive()) {
                    if (!actions.get(0).isSetup()) {
                        actions.get(0).setup();
                    } else {
                        actions.get(0).loop();
                    }
                } else {
                    actions.remove(0);
                }
            }
        } else if (MODE == MULTI) {
            for (int a = 0; a < actions.size(); a++) {
                Action act = actions.get(a);
                if (act.isAlive()) {
                    if (!act.isSetup()) {
                        act.setup();
                    } else {
                        act.loop();
                    }
                }
            }
        }
    }

    void showMessages() {
        for (int s = 0; s < permanent_messages.size(); s++) {
            telemetry.addLine(permanent_messages.get(s));
        }
        showStats();
    }

    void showStats() {
        telemetry.addData("Actions:", actions.size());
        if (isClawOpen()) {
            telemetry.addData("Claw State:", "Opened");
        } else {
            telemetry.addData("Claw State:", "Closed");
        }
        //        telemetry.addData("Seconds", (int) runtime.seconds());
        //        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }

    void handleGamepad1() {
        double turn = getTurnPower();
        if (actions.size() == 0) {
            if (turn != 0) {
                if (turn < 0) {
                    turnLeft(getDrivePower());
                } else {
                    turnRight(getDrivePower());
                }
            } else {
                leftDrive.setPower(getDrivePower());
                rightDrive.setPower(getDrivePower());
            }
        }
        if (gamepad1.b) {
            autoDrive(100);
            buttonSleep();
        } else if (gamepad2.left_stick_x > 0) {
            autoReorientForCube(RIGHT);
            buttonSleep();
        } else if (gamepad2.left_stick_x < 0) {
            autoReorientForCube(LEFT);
            buttonSleep();
        } else if (gamepad1.y) {
            fullAuto();
            buttonSleep();
        } else if (gamepad1.a) {
        }
    }

    void handleGamepad2() {
        armMove(getArmSpeed());
        if (gamepad2.left_bumper) {
            clawOpen();
            buttonSleep();
        } else if (gamepad2.right_bumper) {
            clawClose();
            buttonSleep();
        } else if (gamepad2.a) {
            if (isArmUp()) {
                autoArm(armInitial);
            } else {
                autoArm(armFinal);
            }
        }
        if (gamepad2.y) {
            collapseClaw();
        }
    }

    void checkEmergency() {
        if (gamepad1.x || gamepad2.x) {
            emergencyStop();
            buttonSleep();
        }
    }

    void buttonSleep() {
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    void resetRobot() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetArm();
    }

    void resetArm() {
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void emergencyStop() {
        resetRobot();
        actions.clear();
        telemetry.addLine("Emergency Stop!");
    }

    void callibrateGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Gyro";
        gyro.initialize(parameters);
    }

    void clawOpen() {
        clawLeft.setPosition(0.4);
        clawRight.setPosition(0.4);
    }

    void clawClose() {
        clawLeft.setPosition(0.55);
        clawRight.setPosition(0.55);
    }

    void turnLeft(double forwardPower) {
        if (forwardPower == 0) {
            leftDrive.setPower(getTurnPower());
            rightDrive.setPower(-getTurnPower());
        } else {
            if (forwardPower > 0) {
                leftDrive.setPower(forwardPower / 8);
                rightDrive.setPower(forwardPower);
            } else {
                rightDrive.setPower(forwardPower);
                leftDrive.setPower(forwardPower / 8);
            }
        }
    }

    void turnRight(double forwardPower) {
        if (forwardPower == 0) {
            leftDrive.setPower(getTurnPower());
            rightDrive.setPower(-getTurnPower());
        } else {
            if (forwardPower > 0) {
                leftDrive.setPower(forwardPower);
                //                rightDrive.setPower( forwardPower / 8);
                rightDrive.setPower(forwardPower / 8);
            } else {
                rightDrive.setPower(forwardPower / 8);
                leftDrive.setPower(forwardPower);
            }
        }
    }

    void armMove(double speed) {
        arm.setPower(speed);
    }

    void fullAuto() {
        resetRobot();
        actions.clear();
        MODE = LINEBYLINE;
        actions.addAll(Arrays.asList(getAutonomous()));
    }

    void autoArm(final double toPosition) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition((int) ((tickPerCentimeter / armToGear) * toPosition));
                arm.setPower(0.7);
            }

            @Override
            public boolean onLoop() {
                if (!arm.isBusy()) {
                    resetArm();
                    return true;
                }
                return false;
            }
        }));
    }

    void autoDrive(final int centimeters) {
        actions.add(new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setPower(1);
                rightDrive.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    return true;
                }
                return false;
            }
        }));
    }

    void collapseClaw() {
        clawLeft.setPosition(0.9);
        clawRight.setPosition(0.95);
    }

    void collapseJulinator() {
        julinator.setPosition(0.05);
    }

    void initializeServos() {
        initializeClaw();
        initializeJulinator();
    }

    void initializeClaw() {
        clawLeft = hardwareMap.get(Servo.class, "s1");
        clawRight = hardwareMap.get(Servo.class, "s2");
        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight.setDirection(Servo.Direction.FORWARD);
    }

    void initializeJulinator() {
        julinator = hardwareMap.get(Servo.class, "jul");
        julinator.setDirection(Servo.Direction.REVERSE);
    }

    void initializeMotors() {
        initializeLeftDrive();
        initializeRightDrive();
        initializeArm();
    }

    void initializeLeftDrive() {
        leftDrive = hardwareMap.get(DcMotor.class, "l");
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    void initializeRightDrive() {
        rightDrive = hardwareMap.get(DcMotor.class, "r");
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    void initializeArm() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        armInitial = arm.getCurrentPosition();
        armFinal = armInitial + tickPerArmCentimeter * maxArmCentimeter;
    }
}

class Action {
    @Deprecated
    static class TimeableAction {
        Execute e;
        int time;

        TimeableAction(int time, @Nullable Execute execute) {
            this.time = time;
            this.e = execute;
        }

        Execute getExecutable() {
            return e;
        }

        int getTime() {
            return time;
        }

        interface Execute {
            void onExecute();
        }
    }

    Execute e;

    Action(@NonNull Execute execute) {
        e = execute;
    }

    private boolean alive = true;
    private boolean setup = false;

    boolean isAlive() {
        return alive;
    }

    boolean isSetup() {
        return setup;
    }

    void setup() {
        e.onSetup();
        setup = true;
    }

    void loop() {
        alive = !e.onLoop();
    }

    interface Execute {
        void onSetup();

        boolean onLoop();
    }
}
