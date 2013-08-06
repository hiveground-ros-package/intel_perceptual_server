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
  pipe_line_stop_(true),
  left_gesture_found_(false),
  right_gesture_found_(false)
{
  ui->setupUi(this);
  
  
  pxcStatus sts=PXCSession_Create(&pxc_session_);
  if(sts<PXC_STATUS_NO_ERROR)
  {
    QMessageBox::warning(this, tr("IntelPerceptualServer"), tr("Failed to create an SDK session."));
    return;
  }

  populateDeviceMenu();
  //populateModuleMenu();
  mode_menu_action_group_ = new QActionGroup(this);
  ui->actionLive->setActionGroup(mode_menu_action_group_);
  ui->actionPlayback->setActionGroup(mode_menu_action_group_);
  ui->actionRecord->setActionGroup(mode_menu_action_group_);
  ui->actionLive->setChecked(true);

  resolution_menu_action_group_ = new QActionGroup(this);
  ui->action640x360->setActionGroup(resolution_menu_action_group_);
  ui->action640x480->setActionGroup(resolution_menu_action_group_);
  ui->action1280x720->setActionGroup(resolution_menu_action_group_);

  connect(this, SIGNAL(dataRetrieved(QImage)), this, SLOT(updateUI(QImage)));
  connect(this, SIGNAL(statusChanged(QString)), ui->statusbar, SLOT(showMessage(QString)));

  clear_gesture_image_timer_ = new QTimer(this);
  clear_gesture_image_timer_->start(1000);
  connect(clear_gesture_image_timer_, SIGNAL(timeout()), this, SLOT(clearGestureImage()));


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


      if(ui->checkBoxGeoNode->isChecked())
      {
        int sz = (j == 0) ? 10 : ((geo_nodes_[i][j].radiusImage>5)?(int)geo_nodes_[i][j].radiusImage:5);
        (j < 6 || j == 0) ? ((j == 0) ? pen.setColor(Qt::blue) : pen.setColor(Qt::red)) : ((j == 10) ? pen.setColor(Qt::yellow) : pen.setColor(Qt::green));
        painter.setPen(pen);
        int x, y;
        if(ui->radioButtonDisplayColor->isChecked() || ui->radioButtonDisplayProjection->isChecked())
        {
          x=(int)geo_nodes_color_[i][j].positionImage.x;
          y=(int)geo_nodes_color_[i][j].positionImage.y;
        }
        else
        {
          x=(int)geo_nodes_[i][j].positionImage.x;
          y=(int)geo_nodes_[i][j].positionImage.y;
        }

        painter.drawEllipse(x-sz, y-sz, sz, sz);
      }

      QLabel *label = (i == 0) ? ui->labelGestureLeft : ui->labelGestureRight;
      QPixmap icon_pixmap;
      arm_msg.gesture = gestures_[i].label;
      switch(gestures_[i].label)
      {
        case PXCGesture::Gesture::LABEL_NAV_SWIPE_LEFT:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/swipe_left.png");
          break;
        case PXCGesture::Gesture::LABEL_NAV_SWIPE_RIGHT:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/swipe_right.png");
          break;
        case PXCGesture::Gesture::LABEL_NAV_SWIPE_UP:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/swipe_up.png");
          break;
        case PXCGesture::Gesture::LABEL_NAV_SWIPE_DOWN:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/swipe_down.png");
          break;
        case PXCGesture::Gesture::LABEL_HAND_WAVE:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/wave.png");
          break;
        case PXCGesture::Gesture::LABEL_HAND_CIRCLE:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/circle.png");
          break;
        case PXCGesture::Gesture::LABEL_POSE_THUMB_UP:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/thumb_up.png");
          break;
        case PXCGesture::Gesture::LABEL_POSE_THUMB_DOWN:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/thumb_down.png");
          break;
        case PXCGesture::Gesture::LABEL_POSE_PEACE:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/peace.png");
          break;
        case PXCGesture::Gesture::LABEL_POSE_BIG5:
          if(ui->checkBoxGesture->isChecked()) icon_pixmap.load(":/gestures/big5.png");
          break;
        default:
          if(i == 0)
            left_gesture_found_ = false;
          else
            right_gesture_found_ = false;
          break;
      }

      if(!icon_pixmap.isNull() && ui->checkBoxGesture->isChecked())
      {
        if(i == 0)
          left_gesture_found_ = true;
        else
          right_gesture_found_ = true;
        label->setPixmap(icon_pixmap.scaled(label->width()-2, label->height()-2));
      }

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

    if(ui->checkBoxOpenness->isChecked())
    {
      if (geo_nodes_[0][0].body > 0)
      {
        int y=(100 - geo_nodes_[0][0].openness) * (pixmap.height()-1) / 100;
        if (y>=0 && y<pixmap.height())
        {
          painter.drawLine(pixmap.width()-2, pixmap.height() - 1, pixmap.width()-2, pixmap.height()-y);
        }
      }
      if (geo_nodes_[1][0].body > 0)
      {
        int y=(100 - geo_nodes_[1][0].openness)*(pixmap.height()-1) / 100;
        if (y>=0 && y<pixmap.height())
        {
          painter.drawLine(2, pixmap.height() - 1, 2, pixmap.height()-y);
        }
      }
    }

    //publish only arm with plam
    if (geo_nodes_[i][0].body > 0)
    {
      arms_msg.arms.push_back(arm_msg);
    }
  }


  if(ui->radioButtonDisplayColor->isChecked() || ui->radioButtonDisplayProjection->isChecked())
  {    
    if(ui->checkBoxFace->isChecked() && face_detection_data_.size() != 0)
    {
      pen.setColor(Qt::green);
      painter.setPen(pen);
      for(int i = 0; i < face_detection_data_.size(); i++)
        painter.drawRect(face_detection_data_[i].rectangle.x,
                         face_detection_data_[i].rectangle.y,
                         face_detection_data_[i].rectangle.w,
                         face_detection_data_[i].rectangle.h);
    }
  }
  mutex_.unlock();

  if(arms_publisher_.getNumSubscribers() != 0 && arms_msg.arms.size() != 0)
  {
    arms_msg.header.stamp = ros::Time::now();
    arms_msg.header.frame_id = "intel_perceptual_server";
    arms_publisher_.publish(arms_msg);
  }

  QPixmap result;
  if(ui->checkBoxMirror->isChecked())
  {
    result = pixmap.transformed(QTransform().scale(-1, 1));
  }
  else
  {
    result = pixmap;
  }

  if(!ui->checkBoxScale->isChecked())
    ui->labelImageDisplay->setPixmap(result);
  else
    ui->labelImageDisplay->setPixmap(result.scaled(ui->labelImageDisplay->size(), Qt::KeepAspectRatio));



}

void IntelPerceptualServer::clearGestureImage()
{
  static QPixmap pixmap(":/gestures/none.png");
  if(!left_gesture_found_)
  {
    ui->labelGestureLeft->setPixmap(pixmap.scaled(ui->labelGestureLeft->width()-2,ui->labelGestureLeft->height()-2));
    left_gesture_found_ = true; //toggle
  }

  if(!right_gesture_found_)
  {
    ui->labelGestureRight->setPixmap(pixmap.scaled(ui->labelGestureRight->width()-2,ui->labelGestureRight->height()-2));
    right_gesture_found_ = true; //toggle
  }
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
  //desc.cuids[1]=PXCFaceAnalysis::CUID;
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


  /* Set Module */
  if(ui->action640x360->isChecked())
    pp->EnableImage(PXCImage::COLOR_FORMAT_RGB24, 640, 360);
  if(ui->action640x480->isChecked())
    pp->EnableImage(PXCImage::COLOR_FORMAT_RGB24, 640, 480);
  if(ui->action1280x720->isChecked())
    pp->EnableImage(PXCImage::COLOR_FORMAT_RGB24, 1280, 720);

  pp->EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
  pp->EnableGesture();
  pp->EnableFaceLocation();
  //pp->EnableFaceLandmark();

  /* Init */
  Q_EMIT statusChanged("Init Started");

  bool init_ok = true;

  PXCPoint3DF32 *pos2d = 0;       // array of depth coordinates to be mapped onto color coordinates
  PXCPointF32 *posc = 0;          // array of mapped color coordinates
  pxcF32 dvalues[2];              // special depth values for saturated and low-confidence pixels

  if (pp->Init())
  {

    /* Setup */
    PXCCapture::VideoStream::ProfileInfo pcolor;
    pp->QueryCapture()->QueryVideoStream(0)->QueryProfile(&pcolor);
    PXCCapture::VideoStream::ProfileInfo pdepth;
    pp->QueryCapture()->QueryVideoStream(1)->QueryProfile(&pdepth);

    ROS_INFO("Depth profile: %d %d %x", pdepth.imageInfo.width, pdepth.imageInfo.height, pdepth.imageInfo.format);
    ROS_INFO("Color profile: %d %d %x", pcolor.imageInfo.width, pcolor.imageInfo.height, pcolor.imageInfo.format);

    pxcStatus sts;
    sts = pp->QueryCapture()->QueryDevice()->QueryPropertyAsUID(PXCCapture::Device::PROPERTY_PROJECTION_SERIALIZABLE,&projection_value_);

    if (sts < PXC_STATUS_NO_ERROR)
    {
      ROS_ERROR("Cannot query projection property");
      pp->Close();
      pp->Release();
      return;
    }

    pp->QueryCapture()->QueryDevice()->QueryProperty(PXCCapture::Device::PROPERTY_DEPTH_LOW_CONFIDENCE_VALUE,&dvalues[0]);
    pp->QueryCapture()->QueryDevice()->QueryProperty(PXCCapture::Device::PROPERTY_DEPTH_SATURATION_VALUE,&dvalues[1]);

    ROS_INFO("dvalues %f %f", dvalues[0], dvalues[1]);

    pp->QuerySession()->DynamicCast<PXCMetadata>()->CreateSerializable<PXCProjection>(projection_value_, &projection_);


    PXCSmartPtr<PXCImage> color2;   // the color image after projection
    PXCSmartPtr<PXCAccelerator> accelerator;
    pp->QuerySession()->CreateAccelerator(&accelerator);
    accelerator->CreateImage(&pcolor.imageInfo, 0, 0, &color2);

    int npoints = pdepth.imageInfo.width*pdepth.imageInfo.height;
    pos2d = (PXCPoint3DF32 *)new PXCPoint3DF32[npoints];
    posc = (PXCPointF32 *)new PXCPointF32[npoints];
    int k = 0;
    for (float y=0;y<pdepth.imageInfo.height;y++)
      for (float x=0;x<pdepth.imageInfo.width;x++,k++)
        pos2d[k].x=x, pos2d[k].y=y;

    Q_EMIT statusChanged("Streaming");

    while(!pipe_line_stop_ && !quit_thread_)
    {     
      if (!pp->AcquireFrame(true))
        break;

      PXCGesture *gesture = pp->QueryGesture();
      PXCImage *depth = pp->QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
      PXCImage *rgb = pp->QueryImage(PXCImage::IMAGE_TYPE_COLOR);
      PXCFaceAnalysis *face = pp->QueryFace();

      mutex_.lock();
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_PRIMARY,10,geo_nodes_[0]);
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_SECONDARY,10,geo_nodes_[1]);
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_ELBOW_PRIMARY,&geo_nodes_[0][10]);
      gesture->QueryNodeData(0,PXCGesture::GeoNode::LABEL_BODY_ELBOW_SECONDARY,&geo_nodes_[1][10]);
      gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_PRIMARY,0,&gestures_[0]);
      gesture->QueryGestureData(0,PXCGesture::GeoNode::LABEL_BODY_HAND_SECONDARY,0,&gestures_[1]);

      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 11; j++)
        {
          geo_nodes_color_[i][j] = geo_nodes_[i][j];
          if (geo_nodes_[i][j].body <= 0)
            continue;
          mapXY(geo_nodes_color_[i][j].positionImage.x, geo_nodes_color_[i][j].positionImage.y, depth, &pcolor.imageInfo);
        }
      }


      face_detection_data_.clear();
      PXCFaceAnalysis::Detection *faced=face->DynamicCast<PXCFaceAnalysis::Detection>();
      for (int i=0;;i++)
      {
        int fid;
        if (face->QueryFace(i,&fid)<PXC_STATUS_NO_ERROR) break;
        PXCFaceAnalysis::Detection::Data data;
        if (faced->QueryData(fid,&data)>=PXC_STATUS_NO_ERROR)
          face_detection_data_.push_back(data);
      }
      mutex_.unlock();


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
          image = QImage(depth_data.planes[0], pdepth.imageInfo.width, pdepth.imageInfo.height, QImage::Format_RGB32);
          labelmap->ReleaseAccess(&depth_data);
        }

        if(dispose)
          labelmap->Release();
      }
      else if(ui->radioButtonDisplayColor->isChecked())
      {
        PXCImage::ImageData rgb_data;
        if(rgb->AcquireAccess(PXCImage::ACCESS_READ, pcolor.imageInfo.format, &rgb_data) >= PXC_STATUS_NO_ERROR)
        {
          image = QImage(rgb_data.planes[0], pcolor.imageInfo.width, pcolor.imageInfo.height, QImage::Format_RGB888).convertToFormat(QImage::Format_RGB32).rgbSwapped();
          rgb->ReleaseAccess(&rgb_data);
        }
      }
      else if(ui->radioButtonDisplayProjection->isChecked())
      {
        PXCImage::ImageData ddepth;
        depth->AcquireAccess(PXCImage::ACCESS_READ, &ddepth);
        int dwidth2 = ddepth.pitches[0]/sizeof(pxcU16); // aligned depth width
        if (projection_.IsValid())
        {
          color2->CopyData(rgb);
          PXCImage::ImageData dcolor;
          color2->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::COLOR_FORMAT_RGB24, &dcolor);
          int cwidth2 = dcolor.pitches[0] / 3; // aligned color width

          for (pxcU32 y=0,k=0;y<pdepth.imageInfo.height;y++)
            for (pxcU32 x=0;x<pdepth.imageInfo.width;x++,k++)
              pos2d[k].z=((short*)ddepth.planes[0])[y*dwidth2+x];
          projection_->MapDepthToColorCoordinates(pdepth.imageInfo.width*pdepth.imageInfo.height, pos2d, posc);


          pxcBYTE* ptr;
          for (pxcU32 y=0,k=0;y<pdepth.imageInfo.height;y++)
          {
            for (pxcU32 x=0;x<pdepth.imageInfo.width;x++,k++)
            {
              int xx=(int)(posc[k].x+0.5f), yy= (int) (posc[k].y+0.5f);
              if (xx<0 || yy<0 || xx>=(int) pcolor.imageInfo.width || yy>=(int)pcolor.imageInfo.height) continue;
              if (!ui->checkBoxProjectAll->isChecked() && (pos2d[k].z==dvalues[0] || pos2d[k].z==dvalues[1])) continue; // no mapping based on unreliable depth values
              ptr = dcolor.planes[0] + ((yy*cwidth2)+(xx)) * 3;
              ptr[0] = 0x00;
              ptr[1] = 0x00;
              ptr[2] = 0xFF;
            }
          }

          image = QImage(dcolor.planes[0], pcolor.imageInfo.width, pcolor.imageInfo.height, QImage::Format_RGB888).convertToFormat(QImage::Format_RGB32).rgbSwapped();
          color2->ReleaseAccess(&dcolor);
          depth->ReleaseAccess(&ddepth);
        }
        else
        {
          ROS_WARN("projection is not valid");
        }
      }


      if(!image.isNull())
        Q_EMIT dataRetrieved(image.copy());
      pp->ReleaseFrame();
    }
  }
  else
  {
    Q_EMIT statusChanged("Init Failed");
    init_ok = false;
  }

  pp->Close();
  pp->Release();

  if (pos2d) delete [] pos2d;
  if (posc) delete [] posc;

  if(init_ok)
    Q_EMIT statusChanged("Stopped");
}

void IntelPerceptualServer::mapXY(float &x, float &y, PXCImage* depth, PXCImage::ImageInfo *cinfo)
{
  PXCImage::ImageInfo dinfo;
  depth->QueryInfo(&dinfo);

  PXCImage::ImageData ddata;
  depth->AcquireAccess(PXCImage::ACCESS_READ,&ddata);

  float *uvmap=(float*)ddata.planes[2];
  int index=((int)y)*dinfo.width+x;

  x=uvmap[index*2]*cinfo->width;
  y=uvmap[index*2+1]*cinfo->height;

  depth->ReleaseAccess(&ddata);
}

void IntelPerceptualServer::mapXY(float &x,float &y,float d, PXCCapture::Device *dev, PXCSession *session)
{
   /* Retrieve the PXCProject serializable identifier */
   pxcUID uid;
   dev->QueryPropertyAsUID(PXCCapture::Device::PROPERTY_PROJECTION_SERIALIZABLE, &uid);

   /* Recreate the PXCProject instance */
   PXCSmartPtr<PXCProjection> pj;
   session->DynamicCast<PXCMetadata>()->CreateSerializable<PXCProjection>(uid,&pj);

   /* Map depth to color coordinates */
   PXCPoint3DF32 posd={ x, y, d };
   PXCPointF32   posc;
   pj->MapDepthToColorCoordinates(1,&posd,&posc);
   x=posc.x; y=posc.y;
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
