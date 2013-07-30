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

  connect(this, SIGNAL(dataRetrieved()), this, SLOT(updateUI()));
  connect(this, SIGNAL(statusChanged(QString)), ui->statusbar, SLOT(showMessage(QString)));
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
  pipe_line_future_ = QtConcurrent::run(this, &IntelPerceptualServer::pipeLine);
}

void IntelPerceptualServer::on_pushButtonStop_clicked()
{
  pipe_line_stop_ = true;
  pipe_line_future_.waitForFinished();
  ui->pushButtonStart->setEnabled(true);
  ui->pushButtonStop->setEnabled(false);
}


void IntelPerceptualServer::updateUI()
{
  mutex_.lock();
  QPixmap pixmap = QPixmap::fromImage(last_image_);
  mutex_.unlock();

  ui->labelImageDisplay->setPixmap(pixmap);
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
    QString file_name = QFileDialog::getSaveFileName(0, tr("Save File"),
                                                    "",
                                                    "All file (*.*)");
    if(file_name.length() == 0)
    {
      QMessageBox::warning(this, "Intel Perceptual Server", "No save seleted");
      return;
    }
    pp = new UtilPipeline(0, (pxcCHAR*)file_name.utf16(), true);
    pp->QueryCapture()->SetFilter((pxcCHAR*)device_menu_action_group_->checkedAction()->text().utf16());
  }
  else if(ui->actionPlayback->isChecked())
  {
    QString file_name = QFileDialog::getOpenFileName(0, tr("Open File"), "", "All file (*.*)");
    if(file_name.length() == 0)
    {
      QMessageBox::warning(this, "Intel Perceptual Server", "No file seleted");
      return;
    }
    pp=new UtilPipeline(0,(pxcCHAR*)file_name.utf16(),false);
  }
  else
  {
    pp = new UtilPipeline();
    pp->QueryCapture()->SetFilter((pxcCHAR*)device_menu_action_group_->checkedAction()->text().utf16());
  }
  bool sts=true;

  /* Set Module */
  pp->EnableGesture((pxcCHAR*)module_menu_action_group_->checkedAction()->text().utf16());
  pp->EnableImage(PXCImage::COLOR_FORMAT_RGB32, 640, 480);

  /* Init */
  Q_EMIT statusChanged("Init Started");

  if (pp->Init())
  {
    Q_EMIT statusChanged("Streaming");

    while(!pipe_line_stop_ && !quit_thread_)
    {     
      if (!pp->AcquireFrame(true))
        break;

      PXCGesture *gesture = pp->QueryGesture();
      PXCImage *depth = pp->QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
      PXCImage *rgb = pp->QueryImage(PXCImage::IMAGE_TYPE_COLOR);

      PXCImage::ImageInfo depth_info;
      depth->QueryInfo(&depth_info);

      PXCImage::ImageInfo rgb_info;
      rgb->QueryInfo(&rgb_info);
      ROS_DEBUG_THROTTLE(1.0, "depth_info %d %d %x %d", depth_info.width, depth_info.height, depth_info.format, depth_info.reserved);
      ROS_DEBUG_THROTTLE(1.0, "rgb_info %d %d %x %d", rgb_info.width, rgb_info.height, rgb_info.format, rgb_info.reserved);

      QImage image;
      if(ui->radioButtonDisplayDepth->isChecked() || ui->radioButtonDisplayLabelmap->isChecked())
      {
        PXCImage *labelmap = depth;
        PXCImage::ImageData depth_data;
        bool dispose = false;

        if( ui->radioButtonDisplayLabelmap->isChecked())
        {
          if (gesture->QueryBlobImage(PXCGesture::Blob::LABEL_SCENE,0,&labelmap)<PXC_STATUS_NO_ERROR)
            return;
          dispose = true;
        }


        if(labelmap->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_RGB32, &depth_data) >= PXC_STATUS_NO_ERROR)
        {
          ROS_DEBUG_THROTTLE(1.0, "depth_data %x %d %x", depth_data.format, depth_data.reserved, depth_data.type);
          image = QImage(depth_data.planes[0], depth_info.width, depth_info.height, QImage::Format_RGB32);
          depth->ReleaseAccess(&depth_data);
        }

        if(dispose)
          labelmap->Release();
      }
      else if(ui->radioButtonDisplayColor->isChecked())
      {
        PXCImage::ImageData rgb_data;
        if(rgb->AcquireAccess(PXCImage::ACCESS_READ, rgb_info.format, &rgb_data) >= PXC_STATUS_NO_ERROR)
        {
          ROS_DEBUG_THROTTLE(1.0, "rgb_data %x %d %x", rgb_data.format, rgb_data.reserved, rgb_data.type);
          image = QImage(rgb_data.planes[0], rgb_info.width, rgb_info.height, QImage::Format_RGB888).convertToFormat(QImage::Format_RGB32).rgbSwapped();
          rgb->ReleaseAccess(&rgb_data);
        }
      }

      mutex_.lock();
      last_image_ = image.copy();
      mutex_.unlock();

      Q_EMIT dataRetrieved();

      pp->ReleaseFrame();
    }
  }
  else
  {
    Q_EMIT statusChanged("Init Failed");
    sts=false;
  }

  pp->Close();

  pp->Release();
  if(sts)
    Q_EMIT statusChanged("Stopped");
}

void IntelPerceptualServer::closeEvent(QCloseEvent *event)
{ 
  ROS_INFO("Close!");
  if(!pipe_line_stop_)
  {
    ROS_INFO("Stop!");
    on_pushButtonStop_clicked();
  }

  quit_thread_ = true;
  event->accept();
}
