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
#include <interaction_msgs/Arms.h>

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

  connect(this, SIGNAL(dataRetrieved(QImage)), this, SLOT(updateUI(QImage)));
  connect(this, SIGNAL(statusChanged(QString)), ui->statusbar, SLOT(showMessage(QString)));


  arms_publisher_ = nhp_.advertise<interaction_msgs::Arms>("arms", 1);
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
  if(ui->actionRecord->isChecked())
  {
    if(record_file_name_.length() == 0)
    {
      QMessageBox::warning(this, "Intel Perceptual Server", "No save file seleted");
      return;
    }
  }
  else if(ui->actionPlayback->isChecked())
  {
    if(playback_file_name_.length() == 0)
    {
      QMessageBox::warning(this, "Intel Perceptual Server", "No file seleted");
      return;
    }
  }
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

void IntelPerceptualServer::on_actionPlayback_triggered(bool checked)
{
  if(checked)
    playback_file_name_ = QFileDialog::getOpenFileName(0, tr("Open File"), "", "All file (*.*)");
}

void IntelPerceptualServer::on_actionRecord_triggered(bool checked)
{
  if(checked)
    record_file_name_ = QFileDialog::getSaveFileName(0, "Save File", "", "All file (*.*)");
}


void IntelPerceptualServer::updateUI(const QImage &image)
{
  QPixmap pixmap = QPixmap::fromImage(image);
  QPainter painter(&pixmap);
  QPen pen;  // creates a default pen
  pen.setWidth(3);

  interaction_msgs::Arms arms_msg;

  mutex_.lock();
  for (int i = 0; i < 2; i++)
  {
    interaction_msgs::Arm arm_msg;
    arm_msg.fingers.resize(5);
    arm_msg.arm_id = i;
    for (int j = 0; j < 11; j++)
    {
      if (geo_nodes_[i][j].body <= 0)
        continue;
      int sz = (j == 0) ? 10 : ((geo_nodes_[i][j].radiusImage>5)?(int)geo_nodes_[i][j].radiusImage:5);
      int x=(int)geo_nodes_[i][j].positionImage.x;
      int y=(int)geo_nodes_[i][j].positionImage.y;
      (j < 6 || j == 0) ? ((j == 0) ? pen.setColor(Qt::blue) : pen.setColor(Qt::red)) : ((j == 10) ? pen.setColor(Qt::yellow) : pen.setColor(Qt::green));
      painter.setPen(pen);
      painter.drawEllipse(x-sz, y-sz, sz, sz);

      if(j == 0) //plam
      {
        arm_msg.hand.translation.x = geo_nodes_[i][j].positionWorld.x;
        arm_msg.hand.translation.y = geo_nodes_[i][j].positionWorld.y;
        arm_msg.hand.translation.z = geo_nodes_[i][j].positionWorld.z;
        arm_msg.hand.rotation.w = 1;
      }
      else if(j < 6) //fingers
      {
        arm_msg.fingers[j-1].translation.x = geo_nodes_[i][j].positionWorld.x;
        arm_msg.fingers[j-1].translation.y = geo_nodes_[i][j].positionWorld.y;
        arm_msg.fingers[j-1].translation.z = geo_nodes_[i][j].positionWorld.z;
        arm_msg.fingers[j-1].rotation.w = 1;
      }
      else if(j == 10) //arm
      {
        arm_msg.arm.translation.x = geo_nodes_[i][j].positionWorld.x;
        arm_msg.arm.translation.y = geo_nodes_[i][j].positionWorld.y;
        arm_msg.arm.translation.z = geo_nodes_[i][j].positionWorld.z;
        arm_msg.arm.rotation.w = 1;
      }

    }

    //publish only arm with plam
    if (geo_nodes_[i][0].body > 0)
    {
      arms_msg.arms.push_back(arm_msg);
    }
  }



  for (int i = 0; i < 2; i++)
  {
    QLabel *label = (i == 0) ? ui->labelGestureLeft : ui->labelGestureRight;
    QPixmap icon_pixmap;
    if (gestures_[i].body <= 0)
    {
      continue;
    }

    switch(gestures_[i].label)
    {
      case PXCGesture::Gesture::LABEL_NAV_SWIPE_LEFT:
        icon_pixmap.load(":/gestures/swipe_left.png");
        break;
      case PXCGesture::Gesture::LABEL_NAV_SWIPE_RIGHT:
        icon_pixmap.load(":/gestures/swipe_right.png");
        break;
      case PXCGesture::Gesture::LABEL_NAV_SWIPE_UP:
        icon_pixmap.load(":/gestures/swipe_up.png");
        break;
      case PXCGesture::Gesture::LABEL_NAV_SWIPE_DOWN:
        icon_pixmap.load(":/gestures/swipe_down.png");
        break;
      case PXCGesture::Gesture::LABEL_HAND_WAVE:
        icon_pixmap.load(":/gestures/wave.png");
        break;
      case PXCGesture::Gesture::LABEL_HAND_CIRCLE:
        icon_pixmap.load(":/gestures/circle.png");
        break;
      case PXCGesture::Gesture::LABEL_POSE_THUMB_UP:
        icon_pixmap.load(":/gestures/thumb_up.png");
        break;
      case PXCGesture::Gesture::LABEL_POSE_THUMB_DOWN:
        icon_pixmap.load(":/gestures/thumb_down.png");
        break;
      case PXCGesture::Gesture::LABEL_POSE_PEACE:
        icon_pixmap.load(":/gestures/peace.png");
        break;
      case PXCGesture::Gesture::LABEL_POSE_BIG5:
        icon_pixmap.load(":/gestures/big5.png");
        break;
      default:
        icon_pixmap.load(":/gestures/none.png");
        break;
    }
    if(!icon_pixmap.isNull())
      label->setPixmap(icon_pixmap.scaled(48,48));
  }
  mutex_.unlock();

  if(arms_publisher_.getNumSubscribers() != 0)
  {
    arms_msg.header.stamp = ros::Time::now();
    arms_msg.header.frame_id = "intel_perceptual_server";
    arms_publisher_.publish(arms_msg);
  }

  if(!ui->checkBoxScale->isChecked())
    ui->labelImageDisplay->setPixmap(pixmap);
  else
    ui->labelImageDisplay->setPixmap(pixmap.scaled(ui->labelImageDisplay->size(), Qt::KeepAspectRatio));



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
    pp = new UtilPipeline(0, (pxcCHAR*)record_file_name_.utf16(), true);
    pp->QueryCapture()->SetFilter((pxcCHAR*)device_menu_action_group_->checkedAction()->text().utf16());
  }
  else if(ui->actionPlayback->isChecked())
  {
    pp=new UtilPipeline(0,(pxcCHAR*)playback_file_name_.utf16(),false);
  }
  else
  {
    pp = new UtilPipeline();
    pp->QueryCapture()->SetFilter((pxcCHAR*)device_menu_action_group_->checkedAction()->text().utf16());
  }
  bool sts=true;

  /* Set Module */
  pp->EnableGesture((pxcCHAR*)module_menu_action_group_->checkedAction()->text().utf16());
  pp->EnableImage(PXCImage::COLOR_FORMAT_RGB32, 1280, 720);

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
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_PRIMARY,10,geo_nodes_[0]);
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_SECONDARY,10,geo_nodes_[1]);
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_ELBOW_PRIMARY,&geo_nodes_[0][10]);
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_ELBOW_SECONDARY,&geo_nodes_[1][10]);
      gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_PRIMARY,0,&gestures_[0]);
      gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_SECONDARY,0,&gestures_[1]);
      mutex_.unlock();

      Q_EMIT dataRetrieved(image.copy());
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
  if(!pipe_line_stop_)
  {
    on_pushButtonStop_clicked();
  }
  quit_thread_ = true;
  event->accept();
}
