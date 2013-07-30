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


#ifndef INTEL_PERCEPTUAL_SERVER_H
#define INTEL_PERCEPTUAL_SERVER_H

#include <QMainWindow>
#include <ros/ros.h>
#include <pxcgesture.h>
#include <pxcsmartptr.h>
#include <QMutex>
#include <QFuture>

namespace Ui {
  class MainWindow;
}

class QAction;
class QActionGroup;

class IntelPerceptualServer : public QMainWindow
{
  Q_OBJECT

public:
  explicit IntelPerceptualServer(QMainWindow *parent = 0);
  ~IntelPerceptualServer();
  bool isQuit() { return quit_thread_; }

protected Q_SLOTS:
  void on_pushButtonStart_clicked();
  void on_pushButtonStop_clicked();

  void updateUI();

Q_SIGNALS:
  void dataRetrieved();
  void statusChanged(QString status);

protected:
  void populateDeviceMenu();
  void populateModuleMenu();

  void pipeLine();

protected: //Qt
  void closeEvent(QCloseEvent *evencurrentItemt);

private:
  Ui::MainWindow *ui;
  bool quit_thread_;
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  bool pipe_line_stop_;
  QFuture<void> pipe_line_future_;
  QImage last_image_;
  QMutex mutex_;


  QVector<QAction*> device_menu_actions_;
  QActionGroup *device_menu_action_group_;

  QVector<QAction*> module_menu_actions_;
  QActionGroup *module_menu_action_group_;

  QActionGroup *mode_menu_action_group_;


  PXCSession *pxc_session_;  
};

#endif // INTEL_PERCEPTUAL_SERVER_H
