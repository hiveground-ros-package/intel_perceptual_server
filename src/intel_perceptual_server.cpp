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


#include <intel_perceptual_server/intel_perceptual_server.h>
#include "ui_main_window.h"
#include <QtGui>
#include <QtConcurrentRun>
#include <util_pipeline.h>

IntelPerceptualServer::IntelPerceptualServer(QMainWindow *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  quit_thread_(false),
  nhp_("~"),
  pipe_line_stop_(true)
{
  ui->setupUi(this);
  
  
  pxcStatus sts=PXCSession_Create(&pxc_session_);
  if(sts<PXC_STATUS_NO_ERROR)
  {
    QMessageBox::warning(this, tr("IntelPerceptualServer"), tr("Failed to create an SDK session."));
    return;
  }

  populateDeviceMenu();
  populateModuleMenu();
  mode_menu_action_group_ = new QActionGroup(this);
  ui->actionLive->setActionGroup(mode_menu_action_group_);
  ui->actionPlayback->setActionGroup(mode_menu_action_group_);
  ui->actionRecord->setActionGroup(mode_menu_action_group_);
  ui->actionLive->setChecked(true);

  connect(this, SIGNAL(renderedImage(QImage)), this, SLOT(updateImage(QImage)));
}

IntelPerceptualServer::~IntelPerceptualServer()
{
  pxc_session_->Release();
  delete ui;
}

void IntelPerceptualServer::on_pushButtonStart_clicked()
{
  ui->pushButtonStart->setEnabled(false);
  ui->pushButtonStop->setEnabled(true);
  pipe_line_stop_=false;
  QtConcurrent::run(this, &IntelPerceptualServer::pipeLine);
}

void IntelPerceptualServer::on_pushButtonStop_clicked()
{
  pipe_line_stop_ = true;
  ui->pushButtonStart->setEnabled(true);
  ui->pushButtonStop->setEnabled(false);
}

void IntelPerceptualServer::updateImage(const QImage &image)
{
  ui->labelImageDisplay->setPixmap(QPixmap::fromImage(image));
  //ui->graphicsView->
}

void IntelPerceptualServer::populateDeviceMenu()
{
  device_menu_action_group_ = new QActionGroup(this);

  PXCSession::ImplDesc desc;
  memset(&desc,0,sizeof(desc));
  desc.group=PXCSession::IMPL_GROUP_SENSOR;
  desc.subgroup=PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;
  for (int i=0;;i++) {
    PXCSession::ImplDesc desc1;
    if (pxc_session_->QueryImpl(&desc,i,&desc1) < PXC_STATUS_NO_ERROR)
      break;

    PXCSmartPtr<PXCCapture> capture;
    if (pxc_session_->CreateImpl<PXCCapture>(&desc1,&capture) < PXC_STATUS_NO_ERROR)
      continue;

    for (int j=0;;j++)
    {
      PXCCapture::DeviceInfo dinfo;
      if (capture->QueryDevice(j,&dinfo) < PXC_STATUS_NO_ERROR)
        break;
      QString device_name = QString::fromUtf16((const ushort *)dinfo.name);
      ROS_INFO_STREAM("Found " << device_name.toStdString());
      QAction* action = ui->menuDevice->addAction(device_name);
      device_menu_actions_.push_back(action);
      action->setCheckable(true);
      action->setActionGroup(device_menu_action_group_);
    }
  }
  if(device_menu_actions_.size() != 0)
  {
    device_menu_actions_[0]->setChecked(true);
  }
}

void IntelPerceptualServer::populateModuleMenu()
{
  module_menu_action_group_ = new QActionGroup(this);

  PXCSession::ImplDesc desc, desc1;
  memset(&desc,0,sizeof(desc));
  desc.cuids[0]=PXCGesture::CUID;
  int i;
  for (i=0;;i++)
  {
    if (pxc_session_->QueryImpl(&desc, i, &desc1) < PXC_STATUS_NO_ERROR)
      break;
    QString module_name = QString::fromUtf16((const ushort *)desc1.friendlyName);
    ROS_INFO_STREAM("Found module " << module_name.toStdString());
    QAction* action = ui->menuModule->addAction(module_name);
    module_menu_actions_.push_back(action);
    action->setCheckable(true);
    action->setActionGroup(module_menu_action_group_);
  }
  if(module_menu_actions_.size() != 0)
  {
    module_menu_actions_[0]->setChecked(true);
  }
}

void IntelPerceptualServer::pipeLine()
{
  UtilPipeline *pp = 0;

  /* Set Mode & Source */
  if(ui->actionRecord->isChecked())
  {
    //pp = new UtilPipeline(0,GetRecordFile(),true);
    //pp->QueryCapture()->SetFilter(GetCheckedDevice(hwndDlg));
  }
  else if(ui->actionPlayback->isChecked())
  {
    //pp=new UtilPipeline(0,GetPlaybackFile(),false);
  }
  else
  {
    pp = new UtilPipeline();
    pp->QueryCapture()->SetFilter((pxcCHAR*)device_menu_action_group_->checkedAction()->text().utf16());
  }
  bool sts=true;

  /* Set Module */
  pp->EnableGesture((pxcCHAR*)module_menu_action_group_->checkedAction()->text().utf16());

  /* Init */
  ui->statusbar->showMessage("Init Started");

  if (pp->Init())
  {
    ui->statusbar->showMessage("Streaming");
    disconnected_ = false;

    while(!pipe_line_stop_ && !quit_thread_)
    {
      if (!pp->AcquireFrame(true))
        break;
      PXCGesture *gesture = pp->QueryGesture();
      PXCImage *depth = pp->QueryImage(PXCImage::IMAGE_TYPE_DEPTH);

      PXCImage::ImageInfo info;
      depth->QueryInfo(&info);

      PXCImage::ImageData data;
      if(depth->AcquireAccess(PXCImage::ACCESS_READ,PXCImage::COLOR_FORMAT_RGB32, &data) >= PXC_STATUS_NO_ERROR)
      {
        QImage image(data.planes[0], info.width, info.height, QImage::Format_RGB32);
        Q_EMIT renderedImage(image);
        depth->ReleaseAccess(&data);
      }
      pp->ReleaseFrame();
    }
  }
  else
  {
    ui->statusbar->showMessage("Init Failed");
    sts=false;
  }

  pp->Close();
  pp->Release();
  if(sts)
    ui->statusbar->showMessage("Stopped");

}

void IntelPerceptualServer::closeEvent(QCloseEvent *event)
{
  quit_thread_ = true;
  event->accept();
}
