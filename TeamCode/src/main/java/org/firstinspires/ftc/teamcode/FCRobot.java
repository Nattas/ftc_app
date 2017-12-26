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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
    private ColorSensor color = null;
    private BNO055IMU gyro = null;
    private int armInitial = 0;
    private double armFinal = 0;
    private int MULTI = 1;
    private int LINEBYLINE = 2;
    private int RIGHT = 1;
    private int LEFT = -1;
    private int MODE = MULTI;
    private int BLUE_TEAM = 1;
    private int RED_TEAM = 2;
    private int TEAM = -1;
    private final int GLYPHBOX_NORTHWEST = 3;
    private final int GLYPHBOX_SOUTHWEST = 1;
    private final int GLYPHBOX_NORTHEAST = 4;
    private final int GLYPHBOX_SOUTHEAST = 2;
    private int LOCATION = -1;
    private int ARM_1 = 1;
    private int ARM_2 = 2;
    private int ARM_3 = 3;
    private int ARM_4 = 4;
    private int motorTickPerRevolution = (1440 / 86) * 100;//1674.41
    private int armMotorTickPerRevolution = (1440);
    private double PI = 3.14;
    private double wheelDiameter = 2 * 5.1 * PI;
    private double distanceBetween = 39;
    private double maxArmCentimeter = 65.3;
    private double placeSpinDiameter = 2 * distanceBetween * PI;
    private double placeOutSpinDiameter = distanceBetween * PI;//122.46
    private double tickPerCentimeter = (motorTickPerRevolution) / (wheelDiameter);//52.27
    private double armToGear = 3 / 6.5;
    private double armLength = 28.8;
    private double armCmPerMotorRevolution = (2 * armLength * PI);
    private double tickPerArmCentimeter = (armMotorTickPerRevolution / armToGear) / armCmPerMotorRevolution;
    private double tickPerDegree = (((placeSpinDiameter * tickPerCentimeter) / 360) / 231) * 245;
    private double tickPerDegreeAtPlace = (((placeOutSpinDiameter * tickPerCentimeter) / 360) / 231) * 245;//18.69023569
    private boolean isReady = false;
    private ArrayList<Action> actions = new ArrayList<>();
    private ArrayList<String> permanent_messages = new ArrayList<>();
    private boolean isAutoMotors = false;
    private boolean isAutoParallel = false;

    @Override
    public void init() {
        //        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        initializeMotors();
        initializeServos();
        initializeColorSensor();
        collapseClaw();
        collapseJulinator();
    }

    @Override
    public void init_loop() {
        //        handleModeChange();
        if (!isReady) {
            telemetry.addData("Inst", "Use DPAD to set location (Relative To Cypherbox)");
            telemetry.addData("Inst", "Use A or B to set team color (A-Blue,B-Red)");
            checkAutoMode();
            checkTeamMode();
            isReady = (LOCATION != -1 && TEAM != -1);
            if (isReady) {
                fullAuto();
            }
        } else {
            checkEmergency();
            handleActions();
        }
        showMessages();
    }

    @Override
    public void start() {
        runtime.reset();
        clearMessages();
        clawOpen();
    }

    @Override
    public void loop() {
        checkEmergency();
        handleGamepad1();
        handleGamepad2();
        handleActions();
        showMessages();
        telemetry.addData("Arm-Encoder", arm.getCurrentPosition());
        telemetry.addData("Arm-Target", arm.getTargetPosition());
        telemetry.addData("Jul-Position", julinator.getPosition());
    }

    @Override
    public void stop() {
        collapseClaw();
        resetRobot();
    }

    Action[] getAutonomous(int location) {
        Action[] SW = new Action[]{
                autonomousClawClose(),
                autonomousArmMove(40),
                autonomousJulinatorScan(),
                autonomousWait(500),
                autonomousColorScan(),
                autonomousWait(500),
                autonomousVuforia(GLYPHBOX_SOUTHWEST)
        };
        Action[] SE = new Action[]{
                autonomousClawClose(),
                autonomousArmMove(30),
                autonomousJulinatorScan(),
                autonomousWait(500),
                autonomousColorScan(),
                autonomousWait(500),
                autonomousVuforia(GLYPHBOX_SOUTHEAST)
        };
        Action[] NW = new Action[]{
                autonomousClawClose(),
                autonomousArmMove(40),
                autonomousJulinatorScan(),
                autonomousWait(500),
                autonomousColorScan(),
                autonomousWait(500),
                autonomousVuforia(GLYPHBOX_NORTHWEST)
        };
        Action[] NE = new Action[]{
                autonomousClawClose(),
                autonomousArmMove(30),
                autonomousJulinatorScan(),
                autonomousWait(500),
                autonomousColorScan(),
                autonomousWait(500),
                autonomousVuforia(GLYPHBOX_NORTHEAST)
        };
        switch (location) {
            case GLYPHBOX_SOUTHWEST:
                return SW;
            case GLYPHBOX_SOUTHEAST:
                return SE;
            case GLYPHBOX_NORTHWEST:
                return NW;
            case GLYPHBOX_NORTHEAST:
                return NE;
            default:
                return new Action[0];
        }
    }

    double getDrivePower() {
        double power = 0;
        if (gamepad1.right_trigger != 0) {
            power += gamepad1.right_trigger;
        }
        if (gamepad1.left_trigger != 0) {
            power -= gamepad1.left_trigger;
        }
        if (gamepad1.a) {
            return power / 4;
        } else {
            return power;
        }
    }

    double getTurnPower() {
        double power = gamepad1.left_stick_x;
        if (gamepad1.a) {
            return power / 8;
        } else {
            return power / 2;
        }
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

    boolean isJulinatorNotHome() {
        return julinator.getPosition() > 0.05;
    }

    boolean isJulinatorDown() {
        return isJulinatorNotHome();
    }

    String getLocationString() {
        String location = "Unknown";
        switch (LOCATION) {
            case GLYPHBOX_NORTHEAST:
                location = "Northeast";
                break;
            case GLYPHBOX_NORTHWEST:
                location = "Northwest";
                break;
            case GLYPHBOX_SOUTHEAST:
                location = "Southeast";
                break;
            case GLYPHBOX_SOUTHWEST:
                location = "Southwest";
                break;
        }
        return location;
    }

    String getTeamString() {
        if (TEAM == BLUE_TEAM) {
            return "Blue";
        } else if (TEAM == RED_TEAM) {
            return "Red";
        } else {
            return "No Selected";
        }
    }

    Action autonomousTurnAtPlace(final int direction, final int degree, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                isAutoMotors = true;
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + direction * ((int) (tickPerDegreeAtPlace * degree)));
                leftDrive.setPower(power);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - direction * ((int) (tickPerDegreeAtPlace * degree)));
                rightDrive.setPower(power);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    isAutoMotors = false;
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousTurn(final int direction, final int degree, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                isAutoMotors = true;
                if (direction == RIGHT) {
                    leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                    leftDrive.setPower(power);
                } else {
                    rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerDegree * degree));
                    rightDrive.setPower(power);
                }
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    isAutoMotors = false;
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
                isAutoParallel = true;
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition((int) (arm.getCurrentPosition() + (tickPerArmCentimeter * cm)));
                arm.setPower(1);
            }

            @Override
            public boolean onLoop() {
                if (!arm.isBusy()) {
                    resetArm();
                    isAutoParallel = false;
                    return true;
                }
                return false;
            }
        });
    }

    Action autonomousJulinatorUp() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julinator.setPosition(0);
            }

            @Override
            public boolean onLoop() {
                return !isJulinatorDown();
            }
        });
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

    Action autonomousJulinatorScan() {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                julinator.setPosition(0.37);
            }

            @Override
            public boolean onLoop() {
                return isJulinatorDown();
            }
        });
    }

    Action autonomousColorScan() {
        return new Action(new Action.Execute() {
            ArrayList<Action> miniaction = new ArrayList<>();

            @Override
            public void onSetup() {
                miniaction.add(autonomousWait(500));
                miniaction.add(autonomousTurnAtPlace(RIGHT, 8, 0.3));
                miniaction.add(autonomousJulinatorDown());
                color.enableLed(false);
                boolean isRed = (color.red() > color.blue());
                //                permanent_messages.add("Color: "+color.red()+" "+color.green()+" "+color.blue());
                if (TEAM == RED_TEAM) {
                    if (!isRed) {
                        miniaction.add(autonomousTurnAtPlace(LEFT, 18, 0.2));
                        miniaction.add(autonomousJulinatorUp());
                        miniaction.add(autonomousTurnAtPlace(RIGHT, 10, 0.5));
                    } else {
                        miniaction.add(autonomousTurnAtPlace(RIGHT, 12, 0.2));
                        miniaction.add(autonomousJulinatorUp());
                        miniaction.add(autonomousTurnAtPlace(LEFT, 20, 0.5));
                    }
                } else {
                    if (isRed) {
                        miniaction.add(autonomousTurnAtPlace(LEFT, 18, 0.2));
                        miniaction.add(autonomousJulinatorUp());
                        miniaction.add(autonomousTurnAtPlace(RIGHT, 10, 0.5));
                    } else {
                        miniaction.add(autonomousTurnAtPlace(RIGHT, 12, 0.2));
                        miniaction.add(autonomousJulinatorUp());
                        miniaction.add(autonomousTurnAtPlace(LEFT, 20, 0.5));
                    }
                }
            }

            @Override
            public boolean onLoop() {
                if (miniaction.size() != 0) {
                    if (miniaction.get(0).isAlive()) {
                        if (!miniaction.get(0).isSetup()) {
                            miniaction.get(0).setup();
                        } else {
                            miniaction.get(0).loop();
                        }
                    } else {
                        miniaction.remove(0);
                    }
                }
                return (miniaction.size() == 0);
            }
        });
    }

    Action autonomousVuforia(final int heading) {
        return new Action(new Action.Execute() {
            ArrayList<Action> miniaction = new ArrayList<>();
            OpenGLMatrix lastLocation = null;
            VuforiaLocalizer vuforia;
            RelicRecoveryVuMark mark;
            VuforiaTrackable relicTemplate;
            VuforiaTrackables relicTrackables;
            boolean foundMark = false;

            void initVuforia() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = "AcJU0s7/////AAAAmYAF+R25rU5elxvWG+jzOfkipjv/EqlAWEJ12K0WFESUlxRj3trJYggUrl1cGvVLLpEbh56nvKa7zL9mYbG3P6lcCeUUNtXMKwg7QzPfJBG8HRas8zkQPFahOEgvii83GQCKLihiRGi2UFmlK3RkzVi6NU2d/9v8q1lrFfAT2lJQsqyThtcaYKHbDt9DFQzSTwcQLz7Pblr1cM57P7H3T/XovWdMOZBKeXzT8G00c5J4rRtWmq+Pvk83YFxfAiA22sFPepjkt2eke/QeMiCFE6hwRoq/WlGFsKDhMnGDAy16UlexgrjEQn85vclQxkJh6/Rd7IJ6FF0aCp8ewg29ZV14bWxdr1GEyWwh1BJ50YFQ";
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
                relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
                relicTemplate = relicTrackables.get(0);
                relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
                relicTrackables.activate();
            }

            void searchVuforia() {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    mark = vuMark;
                    foundMark = true;
                    addActions();
                }
            }

            void addActions() {
                if (mark == RelicRecoveryVuMark.CENTER) {
                    switch (heading) {
                        case GLYPHBOX_SOUTHWEST:
                            miniaction.add(autonomousTurnAtPlace(LEFT, 180, 0.4));
                            miniaction.add(autonomousDrive(89, 1));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(32, 0.5));
                            miniaction.add(autonomousArmMove(-20));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_SOUTHEAST:
                            miniaction.add(autonomousTurnAtPlace(LEFT, 180, 0.4));
                            miniaction.add(autonomousDrive(65, 1));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(24, 0.8));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(20, 1));
                            miniaction.add(autonomousArmMove(-15));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_NORTHWEST:
                            miniaction.add(autonomousDrive(89, 1));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(32, 0.5));
                            miniaction.add(autonomousArmMove(-20));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_NORTHEAST:
                            miniaction.add(autonomousDrive(65, 1));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(24, 0.8));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(20, 1));
                            miniaction.add(autonomousArmMove(-15));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                    }
                } else if (mark == RelicRecoveryVuMark.LEFT) {
                    switch (heading) {
                        case GLYPHBOX_SOUTHWEST:
                            miniaction.add(autonomousTurnAtPlace(LEFT, 180, 0.4));
                            miniaction.add(autonomousDrive(110, 1));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(32, 0.5));
                            miniaction.add(autonomousArmMove(-20));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_SOUTHEAST:
                            miniaction.add(autonomousTurnAtPlace(LEFT, 180, 0.4));
                            miniaction.add(autonomousDrive(65, 1));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(5, 0.8));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(20, 1));
                            miniaction.add(autonomousArmMove(-15));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_NORTHWEST:
                            miniaction.add(autonomousDrive(110, 1));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(32, 0.5));
                            miniaction.add(autonomousArmMove(-20));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_NORTHEAST:
                            miniaction.add(autonomousDrive(65, 1));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(5, 0.8));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(20, 1));
                            miniaction.add(autonomousArmMove(-15));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                    }
                } else if (mark == RelicRecoveryVuMark.RIGHT) {
                    switch (heading) {
                        case GLYPHBOX_SOUTHWEST:
                            miniaction.add(autonomousTurnAtPlace(LEFT, 180, 0.4));
                            miniaction.add(autonomousDrive(70, 1));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(32, 0.5));
                            miniaction.add(autonomousArmMove(-20));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_SOUTHEAST:
                            miniaction.add(autonomousTurnAtPlace(LEFT, 180, 0.4));
                            miniaction.add(autonomousDrive(65, 1));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(43, 0.8));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(20, 1));
                            miniaction.add(autonomousArmMove(-15));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_NORTHWEST:
                            miniaction.add(autonomousDrive(70, 1));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(32, 0.5));
                            miniaction.add(autonomousArmMove(-20));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                        case GLYPHBOX_NORTHEAST:
                            miniaction.add(autonomousDrive(65, 1));
                            miniaction.add(autonomousTurnAtPlace(RIGHT, 90, 0.5));
                            miniaction.add(autonomousDrive(43, 0.8));
                            miniaction.add(autonomousTurnAtPlace(LEFT, 90, 0.5));
                            miniaction.add(autonomousDrive(20, 1));
                            miniaction.add(autonomousArmMove(-15));
                            miniaction.add(autonomousClawOpen());
                            miniaction.add(autonomousDrive(-14, 1));
                            break;
                    }
                }
            }

            @Override
            public void onSetup() {
                initVuforia();
            }

            @Override
            public boolean onLoop() {
                if (foundMark) {
                    if (miniaction.size() != 0) {
                        if (miniaction.get(0).isAlive()) {
                            if (!miniaction.get(0).isSetup()) {
                                miniaction.get(0).setup();
                            } else {
                                miniaction.get(0).loop();
                            }
                        } else {
                            miniaction.remove(0);
                        }
                    }
                    return (miniaction.size() == 0);
                } else {
                    searchVuforia();
                    return false;
                }
            }
        });
    }

    Action autonomousWait(final int millis) {
        return new Action(new Action.Execute() {
            int targetTime = 0;

            @Override
            public void onSetup() {
                targetTime = (int) runtime.milliseconds() + millis;
            }

            @Override
            public boolean onLoop() {
                return (runtime.milliseconds() >= targetTime);
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

    Action autonomousDrive(final int centimeters, final double power) {
        return new Action(new Action.Execute() {
            @Override
            public void onSetup() {
                isAutoMotors = true;
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (tickPerCentimeter * centimeters));
                leftDrive.setPower(power);
                rightDrive.setPower(power);
            }

            @Override
            public boolean onLoop() {
                if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
                    resetRobot();
                    isAutoMotors = false;
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
            }

            @Override
            public boolean onLoop() {
                resetRobot();
                MODE = MULTI;
                actions.clear();
                return true;
            }
        });
    }

    void autoTurn(final int direction, final int degree) {
        prepForAuto();
        actions.add(autonomousTurnAtPlace(direction, degree, 1));
        actions.add(autonomousDone());
    }

    void autoArm(final int cm) {
        prepForAuto();
        actions.add(autonomousArmMove(cm));
        actions.add(autonomousDone());
    }

    void autoDrive(final int centimeters) {
        prepForAuto();
        actions.add(autonomousDrive(centimeters, 1));
        actions.add(autonomousDone());
    }

    void prepForAuto() {
        resetRobot();
        actions.clear();
        MODE = LINEBYLINE;
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
        if (MODE == MULTI) {
            telemetry.addData("Mode", "MultiAction");
        } else {
            telemetry.addData("Mode", "ActionByAction");
        }
        //        telemetry.addData("Seconds", (int) runtime.seconds());
        //        telemetry.addData("Motor", leftDrive.getPower() + " " + rightDrive.getPower());
    }

    void clearMessages() {
        permanent_messages.clear();
    }

    void handleGamepad1() {
        double turn = getTurnPower();
        if (!isAutoMotors) {
            if (!rightDrive.isBusy() && !leftDrive.isBusy()) {
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
        }
        if (gamepad1.left_bumper) {
            autoTurn(LEFT, 45);
            buttonSleep();
        } else if (gamepad1.right_bumper) {
            autoTurn(RIGHT, 45);
            buttonSleep();
        } else if (gamepad1.dpad_left) {
            autoTurn(LEFT, 90);
            buttonSleep();
        } else if (gamepad1.dpad_right) {
            autoTurn(RIGHT, 90);
            buttonSleep();
        } else if (gamepad1.dpad_up) {
            autoDrive(20);
            buttonSleep();
        } else if (gamepad1.dpad_down) {
            autoDrive(-20);
            buttonSleep();
        }
    }

    void handleGamepad2() {
        if (!isAutoParallel) {
            if (!arm.isBusy()) {
                armMove(getArmSpeed());
            }
        }
        //        if(gamepad2.left_stick_x!=0){
        //            julinator.setPosition(toServo(-gamepad2.left_stick_x));
        //            buttonSleep();
        //        }
        if (gamepad2.left_bumper) {
            clawOpen();
            buttonSleep();
        } else if (gamepad2.right_bumper) {
            clawClose();
            buttonSleep();
        } else if (gamepad2.y) {
            collapseClaw();
            buttonSleep();
        } else if (gamepad2.dpad_up) {
            autoArm(20);
            buttonSleep();
        } else if (gamepad2.dpad_down) {
            autoArm(-20);
            buttonSleep();
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
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    void resetRobot() {
        //        MODE=MULTI;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetArm();
    }

    void resetArm() {
        arm.setPower(0);
        //        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void emergencyStop() {
        resetRobot();
        actions.clear();
        telemetry.addLine("Emergency Stop!");
    }

    void clawOpen() {
        clawLeft.setPosition(0.3);
        clawRight.setPosition(0.3);
    }

    void clawClose() {
        clawLeft.setPosition(0.55);
        clawRight.setPosition(0.55);
    }

    void jullinatorUp() {
        julinator.setPosition(0);
    }

    void jullinatorDown() {
        julinator.setPosition(0.45);
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

    void checkAutoMode() {
        if (gamepad1.dpad_down && gamepad1.dpad_left) {
            LOCATION = GLYPHBOX_SOUTHWEST;
        } else if (gamepad1.dpad_down && gamepad1.dpad_right) {
            LOCATION = GLYPHBOX_SOUTHEAST;
        } else if (gamepad1.dpad_up && gamepad1.dpad_right) {
            LOCATION = GLYPHBOX_NORTHEAST;
        } else if (gamepad1.dpad_up && gamepad1.dpad_left) {
            LOCATION = GLYPHBOX_NORTHWEST;
        }
        telemetry.addData("Location: ", getLocationString());
    }

    void checkTeamMode() {
        if (gamepad1.a && !gamepad1.b) {
            TEAM = BLUE_TEAM;
        } else if (!gamepad1.a && gamepad1.b) {
            TEAM = RED_TEAM;
        }
        telemetry.addData("Team Color: ", getTeamString());
    }

    void fullAuto() {
        prepForAuto();
        permanent_messages.add("Location: " + getLocationString());
        permanent_messages.add("Team Color: " + getTeamString());
        actions.addAll(Arrays.asList(getAutonomous(LOCATION)));
    }

    void collapseClaw() {
        clawLeft.setPosition(0.6);
        clawRight.setPosition(0.6);
    }

    void collapseJulinator() {
        julinator.setPosition(0);
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
        julinator.setDirection(Servo.Direction.FORWARD);
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
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armInitial = arm.getCurrentPosition();
        armFinal = armInitial + tickPerArmCentimeter * maxArmCentimeter;
    }

    void initializeColorSensor() {
        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(false);
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
