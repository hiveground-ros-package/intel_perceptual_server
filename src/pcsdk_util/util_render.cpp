/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2011-2013 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include <tchar.h>
#include <math.h>
#include "util_render.h"
#include "pxcsmartptr.h"
#include <string>

/// Used by m_scale
static const float DEFAULT_SCALE = 30.0f;

//TODO: distance must be based on camera properties
static const float MAX_Z = 0.5f/13.0f;

UtilRender::UtilRender(pxcCHAR *title) {
  WNDCLASSW wc;
  memset(&wc,0,sizeof(wc));
  wc.lpszClassName=title;
  wc.lpfnWndProc=WindowProc;
  wc.style=CS_HREDRAW|CS_VREDRAW;
  wc.hCursor=LoadCursor(NULL, IDC_ARROW);
  RegisterClassW(&wc);

  m_bitmap=0;
  m_fps_nframes=0;
  wcscpy_s<1024>(m_title,title);
  m_frame=0;
  m_pause=0;

  m_hWnd=CreateWindowW(title,title,WS_BORDER|WS_CAPTION|WS_SYSMENU|WS_SIZEBOX,CW_USEDEFAULT,CW_USEDEFAULT,100,100,0,0,0,this);
  memset(&m_info,0,sizeof(m_info));
  m_buffer=0;
  m_scale=DEFAULT_SCALE;
  m_mouse.x=m_mouse.y=(pxcU32)-1;
  m_depth_mode = RENDER_Z;
}

UtilRender::~UtilRender(void) {
  if (m_hWnd) DestroyWindow(m_hWnd);
  if (m_bitmap) DeleteObject(m_bitmap);
  if (m_buffer) delete [] m_buffer;
}

void UtilRender::SetSize(pxcU32 width, pxcU32 height) {
  if (m_buffer) delete [] m_buffer;
  m_buffer=new pxcU32[width*height];

  m_info.bmiHeader.biWidth=width;
  m_info.bmiHeader.biHeight= - (int)height;
  m_info.bmiHeader.biBitCount=32;
  m_info.bmiHeader.biPlanes=1;
  m_info.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);
  m_info.bmiHeader.biCompression=BI_RGB;

  // fit into the client rect
  WINDOWINFO winInfo;
  GetWindowInfo(m_hWnd, &winInfo);
  int borderX = (winInfo.rcWindow.right-winInfo.rcWindow.left) - (winInfo.rcClient.right - winInfo.rcClient.left);
  int borderY = (winInfo.rcWindow.bottom-winInfo.rcWindow.top) - (winInfo.rcClient.bottom - winInfo.rcClient.top);
  SetWindowPos(m_hWnd,0,0,0, width+borderX, height+borderY, SWP_NOMOVE|SWP_NOZORDER|SWP_NOREDRAW|SWP_SHOWWINDOW);
}

static pxcU32 Float2Color(float value) 
{
  pxcU32 color = 0xff000000;
  if (value <= 0.f || value > 1.f) return color;

  float saturation = 0.3f + 0.7f*(1.f-value);
  float hue = value*6; // hue
  float V = 1.f-pow(value,3); // brightness
  float F = hue-(int)hue;
  float M = V*(1-saturation);
  float N = V*(1-saturation*F);
  float K = V*(1-saturation*(1-F));
  float r, g, b;
  switch ((int)hue) {
  case 0: r=V; g=K; b=M; break;
  case 1: r=N; g=V; b=M; break;
  case 2: r=M; g=V; b=K; break;
  case 3: r=M; g=N; b=V; break;
  case 4: r=K; g=M; b=V; break;
  case 5: r=V; g=M; b=N; break;
  default: r=g=b=0;
  }

  unsigned char* rgb = (unsigned char*) &color;
  rgb[0] = (unsigned char)(255.f*r);
  rgb[1] = (unsigned char)(255.f*g);
  rgb[2] = (unsigned char)(255.f*b);
  return color;
}

void SolidEdges(pxcI16* pSrc, int channels, pxcU32* pDst, int w, int h, float scale)
{
  pxcI16* src = pSrc + (channels-1);
  pxcU32* dst = pDst;

  float minZ = 0;
  float maxZ = MAX_Z;

  for(int i=0;i<h;i++) {
    for(int j=0;j<w;j++) {
      float z0 = (float)src[0] / 32768;
      float z1 = j>0   ? (float)src[-channels] / 32768 : 0;
      float z2 = j<w-1 ? (float)src[ channels] / 32768 : 0;
      float z3 = i>0   ? (float)src[-w*channels] / 32768 : 0;
      float z4 = i<h-1 ? (float)src[ w*channels] / 32768 : 0;
      if (abs(z0-z1)>scale || abs(z0-z2)>scale || abs(z0-z3)>scale || abs(z0-z4)>scale) {
        *dst = 0;
      } else {
        float val = (z0-minZ)/(maxZ-minZ);
        *dst = Float2Color(val);
      }
      src+=channels;
      dst++;
    }
  }
}

void ConfidenceMap(pxcI16* pSrc, pxcU32* pDst, int w, int h)
{
  pxcI16* src = pSrc;
  pxcU32* dst = pDst;

  float minC = 1000.0f;
  float maxC = -1000.0f;

  for(int k=0;k<w*h;k++)
  {
    float vC = (float)src[0] / 32768;
    if (minC>vC) minC = vC;
    if (maxC<vC) maxC = vC;
    src++;
  }

  src = pSrc;
  for(int i=0;i<w*h;i++)
  {
    float val = (float)src[0] / 32768;
    val = (val-minC)/(maxC-minC);
    unsigned char* rgb = (unsigned char*)dst;
    rgb[0] = (unsigned char)(255.f*val);
    rgb[1] = (unsigned char)(255.f*val);
    rgb[2] = (unsigned char)(255.f*val);
    src++;
    dst++;
  }
}

bool UtilRender::RenderFrame(PXCImage *image) {
  if (!image) return false;
  if (m_bitmap) {
    DeleteObject(m_bitmap);
  } else {
    if (!m_info.bmiHeader.biWidth || !m_info.bmiHeader.biHeight) {
      PXCImage::ImageInfo info;
      image->QueryInfo(&info);
      SetSize(info.width, info.height);
    }
  }
  if (!m_pause) m_frame++;

  /* Calcualte FPS */
  if ((m_fps_nframes++)==0) {
    LARGE_INTEGER now, freq;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&freq);
    m_fps_first=(double)now.QuadPart/(double)freq.QuadPart;
  }
  if (m_fps_nframes>30) {
    LARGE_INTEGER now, freq;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&freq);
    int fps=(int)((double)m_fps_nframes/((double)now.QuadPart/(double)freq.QuadPart-m_fps_first));

    pxcCHAR line[1024];
    swprintf_s<1024>(line,L"%s (%d fps)",m_title, fps);
    SetWindowTextW(m_hWnd,line);
    m_fps_nframes=0;
  }

  if (m_buffer) {
    PXCImage::ImageInfo info;
    image->QueryInfo(&info);
    PXCImage::ImageData data;
    //TODO give distance for the camera properties
    float max_distance = MAX_Z;
    float range_min = .0f;
    float range_max = max_distance * m_scale / DEFAULT_SCALE;
    if( info.format ==  PXCImage::COLOR_FORMAT_VERTICES )
    {
      pxcStatus sts=image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_VERTICES, &data);
      if (sts>=PXC_STATUS_NO_ERROR)
      {
        pxcU32* dst = m_buffer;
        pxcI16* src = (pxcI16*)data.planes[0];
        switch(m_depth_mode)
        {
        case RENDER_CONFIDENCE: // confidence map in gray scale
          if (data.planes[1])
          {
            ConfidenceMap( (pxcI16*)data.planes[1], dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight);
            break;
          }
          m_depth_mode = RENDER_EDGES;
        case RENDER_EDGES: // Z coordinate in color with edge detection
          //                    SolidEdgesVertices( src, dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight, m_scale*0.001f*max_distance);
          SolidEdges( src, 3, dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight, m_scale*0.001f*max_distance);
          break;
        case RENDER_X: // X coordinate in color
        case RENDER_Y: // Y coordinate in color
          for(int j=0; j<-m_info.bmiHeader.biHeight * m_info.bmiHeader.biWidth; j++)
          {
            float val = (float)src[m_depth_mode == RENDER_X ? 0 : 1] / 32768;
            val = (val-range_min)/(range_max-range_min);
            *dst = Float2Color(abs(val));
            dst++;
            src += 3;
          }
          break;
        case RENDER_Z: // Z coordinate in gray scale
          for(int j=0; j<-m_info.bmiHeader.biHeight * m_info.bmiHeader.biWidth; j++)
          {
            float val = (float)src[2] / 32768;
            val = 1.f - (val-range_min)/(range_max-range_min);
            val = max(val, 0.f);
            val = min(val, 1.f);
            val*=val; // use square
            unsigned char* rgb = (unsigned char*) dst;
            rgb[0] = (unsigned char)(255.f*val);
            rgb[1] = (unsigned char)(255.f*val);
            rgb[2] = (unsigned char)(255.f*val);
            rgb[3] = 255;
            dst++;
            src += 3;
          }
          break;
        }
        sts=image->ReleaseAccess(&data);
      }
    } else if (info.format == PXCImage::COLOR_FORMAT_DEPTH && IsRenderingNonVertex())
    {
#if 0
      // This implementation forces distance plane data to only render distance values
      // Vertex data can still render confidence and edges
      pxcStatus sts=image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_RGB32, &data);
      if (sts>=PXC_STATUS_NO_ERROR) {
        for(int i=0;i<-m_info.bmiHeader.biHeight;i++)
          memcpy_s((char*)(m_buffer+i*m_info.bmiHeader.biWidth),m_info.bmiHeader.biWidth*4,data.planes[0]+i*data.pitches[0],m_info.bmiHeader.biWidth*4);
        sts=image->ReleaseAccess(&data);
      }
#else
      pxcStatus sts=image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_DEPTH, &data);
      if (sts>=PXC_STATUS_NO_ERROR)
      {
        pxcU32* dst = m_buffer;
        pxcI16* src = (pxcI16*)data.planes[0];
        switch(m_depth_mode)
        {
        case RENDER_CONFIDENCE: // confidence map in gray scale
          if (data.planes[1])
          {
            ConfidenceMap( (pxcI16*)data.planes[1], dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight);
            break;
          }
          m_depth_mode = RENDER_EDGES;
        case RENDER_EDGES: // Z coordinate in color with edge detection
          SolidEdges(src, 1, dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight, m_scale*0.001f*max_distance);
          break;
        }
        sts=image->ReleaseAccess(&data);
      }
#endif
    } else
    {
      pxcStatus sts=image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::COLOR_FORMAT_RGB32, &data);
      if (sts>=PXC_STATUS_NO_ERROR) {
        for(int i=0;i<-m_info.bmiHeader.biHeight;i++)
          memcpy_s((char*)(m_buffer+i*m_info.bmiHeader.biWidth),m_info.bmiHeader.biWidth*4,data.planes[0]+i*data.pitches[0],m_info.bmiHeader.biWidth*4);
        sts=image->ReleaseAccess(&data);
      }
    }

    HDC dc=GetDC(m_hWnd);
    m_bitmap=CreateDIBitmap(dc, &m_info.bmiHeader, CBM_INIT, m_buffer, &m_info, DIB_RGB_COLORS);
    ReleaseDC(m_hWnd, dc);
    InvalidateRect(m_hWnd, 0, FALSE);
  }

  DoMessageLoop();
  return IsWindowVisible(m_hWnd)?true:false;
}

void UtilRender::DoMessageLoop(void) {
  for (MSG msg; PeekMessage(&msg, m_hWnd,  0, 0, PM_REMOVE);) {
    DispatchMessage(&msg);
  }
}

bool UtilRender::RenderFrame(PXCAudio *audio) {
  if (m_bitmap) {
    DeleteObject(m_bitmap);
    m_bitmap = NULL;
  } else {
    if (!m_info.bmiHeader.biWidth || !m_info.bmiHeader.biHeight) {
      PXCAudio::AudioInfo info;
      audio->QueryInfo(&info);
      m_bufferTime = audio->QueryTimeStamp();
      m_numSamples = 0;
      int channel_h = 160;
      int w = 640;
      int h = channel_h*info.nchannels;
      SetSize(w, h);
      m_scale = 0.4f*channel_h;
    }
  }

  if(m_buffer) {
    while (DrawAudioWave(audio)) {
      if (m_bitmap) {
        DeleteObject(m_bitmap);
        m_bitmap = NULL;
      }
      HDC dc=GetDC(m_hWnd);
      m_bitmap=CreateDIBitmap(dc, &m_info.bmiHeader, CBM_INIT, m_buffer, &m_info, DIB_RGB_COLORS);
      ReleaseDC(m_hWnd, dc);
      InvalidateRect(m_hWnd, 0, FALSE);
      DoMessageLoop();
    }
  }

  return IsWindowVisible(m_hWnd)?true:false;
}

LRESULT UtilRender::WindowProc(HWND hwnd,UINT uMsg,WPARAM wParam,LPARAM lParam) {
  UtilRender *pthis=(UtilRender*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

  switch (uMsg) {
  case WM_CREATE:
    SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)((CREATESTRUCT*)lParam)->lpCreateParams);
    return TRUE;
  case WM_PAINT:
    if (pthis->m_bitmap) {
      PAINTSTRUCT ps;
      HDC hdc = BeginPaint(hwnd, &ps);
      HDC hdc2= CreateCompatibleDC(hdc);
      SelectObject(hdc2, pthis->m_bitmap);
      RECT rect;
      GetClientRect(hwnd, &rect);
      //SetStretchBltMode(hdc, HALFTONE); // slow but quality
      SetStretchBltMode(hdc, BLACKONWHITE);
      StretchBlt(hdc,0,0,rect.right-rect.left, rect.bottom-rect.top, hdc2, 0, 0, pthis->m_info.bmiHeader.biWidth,-pthis->m_info.bmiHeader.biHeight,SRCCOPY);
      DeleteObject(hdc2); // Calling DeleteObject instead of DeleteDC since DeleteDC results in a GDI resource leak.
      pthis->DrawMore(hdc,(double)(rect.right-rect.left)/pthis->m_info.bmiHeader.biWidth,(double)(rect.bottom-rect.top)/(-pthis->m_info.bmiHeader.biHeight));
      EndPaint(hwnd, &ps);
    }
    return TRUE;
  case WM_MOUSEWHEEL:
    if (GET_WHEEL_DELTA_WPARAM(wParam)>0) pthis->m_scale*=1.5; else pthis->m_scale/=1.5;
    return TRUE;
  case WM_MOVE:
    pthis->m_fps_nframes=0;
    return TRUE;
  case WM_LBUTTONDOWN:
    pthis->m_mouse.x=LOWORD(lParam);
    pthis->m_mouse.y=HIWORD(lParam);
    return FALSE;
  case WM_KEYDOWN:
    switch (wParam)
    {
    case VK_F1: // X coordinates in color (for vertices windows)
      pthis->m_depth_mode = RENDER_X;
      break;
    case VK_F2: // Y coordinates in color (for vertices windows)
      pthis->m_depth_mode = RENDER_Y;
      break;
    case VK_F3: // Z coordinates in gray scale (for vertices windows)
      pthis->m_depth_mode = RENDER_Z;
      pthis->m_scale = DEFAULT_SCALE;
      break;
    case VK_F5: // Z coordinate in color with edge detection
      pthis->m_depth_mode = RENDER_EDGES;
      pthis->m_scale = DEFAULT_SCALE;
      break;
    case VK_F6: // confidence map in gray scale
      pthis->m_depth_mode = RENDER_CONFIDENCE;
      break;
    case VK_RIGHT: // right - next frame
      pthis->m_frame++;
      pthis->m_pause=1;
      break;
    case VK_LEFT: // left - previous frame
      pthis->m_frame--;
      if (pthis->m_frame<0) pthis->m_frame=0;
      pthis->m_pause=1;
      break;
    case VK_SPACE: // space - unpause
      pthis->m_pause=!pthis->m_pause;
      break;
    }
    return FALSE;
  case WM_CLOSE:
    ShowWindow(hwnd,SW_HIDE);
    return FALSE;
  default:
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
  }
}

bool UtilRender::ResampleAudio(PXCAudio *audio) {
  PXCAudio::AudioInfo info;
  audio->QueryInfo(&info);
  PXCAudio::AudioData data;
  pxcStatus sts=audio->AcquireAccess(PXCAudio::ACCESS_READ,PXCAudio::AUDIO_FORMAT_IEEE_FLOAT,&data);
  if (sts < PXC_STATUS_NO_ERROR) {
    return false;
  }
  double srcDuration = 10000000. / info.sampleRate;
  double dstDuration = 10000.;

  int nsamples = (int)(data.dataSize/info.nchannels);
  double timeSrc = (double)audio->QueryTimeStamp() - (double)m_bufferTime;
  double timeDst = dstDuration*m_numSamples;
  float* dstStream = m_sampleBuffer + m_numSamples*info.nchannels;
  float *srcStream = (float*)data.dataPtr;

  int ind;
  bool ret = false;
  while ((ind=(int)((timeDst-timeSrc)/srcDuration)) < nsamples) {
    if (ind<0) { // lost packets compensation
      timeDst = timeSrc;
    } else {
      srcStream += info.nchannels*ind;
      timeSrc += srcDuration*ind;
      nsamples -= ind;
    }
    memcpy_s(dstStream, info.nchannels*sizeof(float), srcStream, info.nchannels*sizeof(float));
    if (++m_numSamples==AUDIO_SAMPLE_COUNT) {
      m_numSamples = 0;
      m_bufferTime += (pxcU64)timeDst;
      ret = true; // draw buffer
      break;
    }
    dstStream += info.nchannels;
    timeDst += dstDuration;
  }
  audio->ReleaseAccess(&data);
  return ret; // non-completed buffer
}

bool UtilRender::DrawAudioWave(PXCAudio *audio) {

  if (!ResampleAudio(audio)) return false;

  PXCAudio::AudioInfo info;
  audio->QueryInfo(&info);
  int w = m_info.bmiHeader.biWidth;
  int h = -m_info.bmiHeader.biHeight;

  /* shift the screen to the left nsamples pixels */
  for (int y=0;y<h;y++) {
    memcpy_s(m_buffer+y*w, (w-AUDIO_SAMPLE_COUNT)*sizeof(pxcU32), m_buffer+y*w+AUDIO_SAMPLE_COUNT, (w-AUDIO_SAMPLE_COUNT)*sizeof(pxcU32));
    memset(m_buffer+(y+1)*w-AUDIO_SAMPLE_COUNT, 0, AUDIO_SAMPLE_COUNT*sizeof(pxcU32));
  }

  int hh  = h/info.nchannels;
  int hh2 = hh/2;
  pxcU32 *base = m_buffer;
  pxcU32 axis_color[4] = {0xffffff00, 0xffff0000, 0xff00ff00, 0xff0000ff};
  for (int c=0; c<(int)info.nchannels; c++) {
    for (int x=0; x<AUDIO_SAMPLE_COUNT; x++) {
      int y = (int)(m_sampleBuffer[x*info.nchannels+c]*m_scale);

      // truncate value
      if (y>hh2)       y = hh2;
      else if (y<-hh2) y = -hh2;

      // draw column
      if (y>0) {
        for (int i=1; i<y; i++) {
          base[(i+hh2)*w-AUDIO_SAMPLE_COUNT+x]=(pxcU32)0xffffffff;
        }
      } else {
        for (int i=-1; i>y; i--) {
          base[(i+hh2)*w-AUDIO_SAMPLE_COUNT+x]=(pxcU32)0xffffffff;
        }
      }

      // draw axis point
      base[hh2*w-AUDIO_SAMPLE_COUNT+x] = axis_color[c%4];
    }
    base += hh*w;
  }

  return true;
}

int UtilRender::QueryFormatSize(PXCAudio::AudioFormat format)
{
  return (format & PXCAudio::AUDIO_FORMAT_SIZE_MASK) >> 3;
}

