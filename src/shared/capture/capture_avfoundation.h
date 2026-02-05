//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*! \file    capture_avfoundation.h
 *  \brief   macOS capture backend using AVFoundation
 */
//========================================================================

#ifndef CAPTURE_AVFOUNDATION_H
#define CAPTURE_AVFOUNDATION_H

// For MOC, we need to expose the class definition without platform-specific includes
#ifdef Q_MOC_RUN

#include "captureinterface.h"
#include "timer.h"

#include <climits>
#include <deque>
#include <string>
#include <vector>

#include <QMutex>
#include <QWaitCondition>
#include <QObject>

// Forward declarations for Objective-C types
typedef struct objc_object AVCaptureSession;
typedef struct objc_object AVCaptureDeviceInput;
typedef struct objc_object AVCaptureVideoDataOutput;
typedef struct objc_object CaptureAVFoundationFrameDelegate;
typedef struct objc_object AVCaptureDevice;
struct CMSampleBuffer;

#else // !Q_MOC_RUN

#ifdef __APPLE__

#include "captureinterface.h"
#include "timer.h"

#include <CoreMedia/CoreMedia.h>
#include <dispatch/dispatch.h>
#include <objc/objc.h>

#include <climits>
#include <deque>
#include <string>
#include <vector>

#include <QMutex>
#include <QWaitCondition>
#include <QObject>

#ifndef __OBJC__
typedef struct objc_object AVCaptureSession;
typedef struct objc_object AVCaptureDeviceInput;
typedef struct objc_object AVCaptureVideoDataOutput;
typedef struct objc_object CaptureAVFoundationFrameDelegate;
typedef struct objc_object AVCaptureDevice;
#else
@class AVCaptureSession;
@class AVCaptureDeviceInput;
@class AVCaptureVideoDataOutput;
@class CaptureAVFoundationFrameDelegate;
@class AVCaptureDevice;
#endif

#endif // __APPLE__

#endif // Q_MOC_RUN

// Only define the class on Apple platforms or when running MOC
#if defined(__APPLE__) || defined(Q_MOC_RUN)

class CaptureAVFoundation : public QObject, public CaptureInterface {
  Q_OBJECT
public:
  explicit CaptureAVFoundation(VarList *settings, QObject *parent = nullptr);
  ~CaptureAVFoundation() override;

  bool startCapture() override;
  bool stopCapture() override;
  bool isCapturing() override { return is_capturing; }
  RawImage getFrame() override;
  void releaseFrame() override;
  void readAllParameterValues() override;
  string getCaptureMethodName() const override { return "AVFoundation"; }

  // Called by Objective-C delegate
  void handleSampleBuffer(CMSampleBufferRef sampleBuffer);

private:
  struct CapturedFrame {
    std::vector<unsigned char> pixels;
    int width = 0;
    int height = 0;
    double timestamp = 0.0;
    double timestamp_cam = 0.0;
  };

  void refreshDevices();
  bool configureSession();
  void teardownSession();
  bool ensureDeviceSelection();
  bool ensureAuthorization();

  bool is_capturing = false;

  VarList *capture_settings = nullptr;
  VarStringEnum *v_device = nullptr;
  VarInt *v_width = nullptr;
  VarInt *v_height = nullptr;
  VarInt *v_fps = nullptr;
  VarBool *v_mirror = nullptr;
  VarBool *v_diagnostics = nullptr;

  QMutex frame_mutex;
  QWaitCondition frame_available;
  std::deque<CapturedFrame> frame_queue;
  RawImage frame_buffer;

  AVCaptureSession *session = nullptr;
  AVCaptureDeviceInput *device_input = nullptr;
  AVCaptureVideoDataOutput *video_output = nullptr;
  CaptureAVFoundationFrameDelegate *delegate = nullptr;
  dispatch_queue_t capture_queue = nullptr;

  bool applyFormatPreferences(AVCaptureDevice *device);
  bool diagnosticsEnabled() const;
  void logDiagnostic(const std::string &message) const;
};

#endif // defined(__APPLE__) || defined(Q_MOC_RUN)

#endif // CAPTURE_AVFOUNDATION_H
