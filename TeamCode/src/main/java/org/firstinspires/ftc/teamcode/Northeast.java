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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Northeast")
public class Northeast extends com.qualcomm.robotcore.eventloop.opmode.OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Robot r;

    @Override
    public void init() {
        r = new Robot(hardwareMap, runtime, telemetry);
        r.init();
        fullAuto();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        r.peel();
    }

    @Override
    public void loop() {
        checkEmergency();
        r.loop();
    }

    @Override
    public void stop() {
    }

    Action[] getAutonomous() {
        return new Action[]{
//                r.autonomousArmMove(Stats.armUp, 0.5),
                r.autonomousJulieScanPosition(),
                r.autonomousWait(500),
                r.autonomousJulieColorScan(Stats.RED_TEAM),
                r.autonomousWait(500),
                r.autonomousVuforia(getScenario()),
                r.autonomousDone()
        };
    }

    Robot.Scenario getScenario() {
        return new Robot.Scenario(
                new Action[]{
                }, new Action[]{
        }, new Action[]{
//                r.autonomousDrive(Stats.westR, 0.5)
        });
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

    void fullAuto() {
        r.prepareForAutonomous();
        r.addActions(getAutonomous());
    }
}