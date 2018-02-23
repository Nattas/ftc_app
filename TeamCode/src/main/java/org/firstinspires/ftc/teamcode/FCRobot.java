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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FullControl", group = "TeleOp")
public class FCRobot extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot r;

    @Override
    public void init() {
        r= new Robot(hardwareMap, runtime, telemetry);
        r.init();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        r.peel();
//        r.clawOpen();
    }

    @Override
    public void loop() {
        checkEmergency();
        handleGamepad1();
        handleGamepad2();
        r.loop();
    }

    @Override
    public void stop() {
//        r.clawClose();
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
            speed -= gamepad2.left_trigger;
        }
        return speed * 0.7;
    }

    void handleGamepad1() {
        double turn = getTurnPower();
        if (!r.isRobotBusy()) {
            if (turn != 0) {
                r.turn(getDrivePower(), turn);
            } else {
                r.drive(getDrivePower());
            }
        }
        if(gamepad1.y){
            r.drifttt();
        }
    }

    void handleGamepad2() {
        if (!r.isRobotBusy()) {
//            r.setLimitArm(!gamepad2.b);
            r.arm(getArmSpeed());
        }
        //        if(gamepad2.left_stick_x!=0){
        //            julinator.setPosition(toServo(-gamepad2.left_stick_x));
        //            buttonSleep();
        //        }
        if (gamepad2.left_bumper) {
            r.pump(1);
        } else if (gamepad2.right_bumper) {
            r.pump(-1);
        } else if (gamepad2.y) {
            r.julieUp();
            buttonSleep();
        }
    }

    void checkEmergency() {
        if (gamepad1.x || gamepad2.x) {
            r.emergency();
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
}
