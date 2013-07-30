/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 *
 */


#include <ros/ros.h>
#include <QtGui/QApplication>
#include <intel_perceptual_server/intel_perceptual_server.h>
#include <QtConcurrentRun>



IntelPerceptualServer* g_app = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_app->isQuit())
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "intel_perceptual_server", ros::init_options::NoSigintHandler);
  QFuture<void> future = QtConcurrent::run(spin_function);


  QApplication a(argc, argv);
  IntelPerceptualServer app;
  g_app = & app;
  app.show();

  g_initialized = true;
  int ret = a.exec();

  future.waitForFinished();

  return ret;
}
