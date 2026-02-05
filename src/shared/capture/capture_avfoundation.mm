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
/*!
  \file    capture_avfoundation.mm
  \brief   C++ / Objective-C++ Implementation: CaptureAVFoundation
*/
//========================================================================

#include "capture_avfoundation.h"

#ifdef __APPLE__

#import <AVFoundation/AVFoundation.h>
#import <Accelerate/Accelerate.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>
#import <dispatch/dispatch.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>

#include <QString>
#include <QMutexLocker>

#include "colors.h"
#include "rawimage.h"

namespace {
constexpr double kDefaultFPS = 60.0;
constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;
constexpr size_t kMaxBufferedFrames = 3;
constexpr int kFrameWaitTimeoutMs = 500;
}

@interface CaptureAVFoundationFrameDelegate : NSObject<AVCaptureVideoDataOutputSampleBufferDelegate>
@property(nonatomic, assign) CaptureAVFoundation *owner;
@end

@implementation CaptureAVFoundationFrameDelegate
- (void)captureOutput:(AVCaptureOutput *)output
 didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
        fromConnection:(AVCaptureConnection *)connection {
  (void)output;
  (void)connection;
  if (self.owner) {
    self.owner->handleSampleBuffer(sampleBuffer);
  }
}
@end

CaptureAVFoundation::CaptureAVFoundation(VarList *settings, QObject *parent)
    : QObject(parent), CaptureInterface(settings) {
  capture_settings = new VarList("AVFoundation");
  settings->addChild(capture_settings);

  v_device = new VarStringEnum("device", "Default");
  v_device->addFlags(VARTYPE_FLAG_AUTO_EXPAND_TREE);
  capture_settings->addChild(v_device);

  v_width = new VarInt("width", kDefaultWidth, 160, 4096);
  v_height = new VarInt("height", kDefaultHeight, 120, 2160);
  v_fps = new VarInt("fps", static_cast<int>(kDefaultFPS), 1, 240);
  v_mirror = new VarBool("mirror", false);
  v_diagnostics = new VarBool("diagnostics", false);

  capture_settings->addChild(v_width);
  capture_settings->addChild(v_height);
  capture_settings->addChild(v_fps);
  capture_settings->addChild(v_mirror);
  capture_settings->addChild(v_diagnostics);

  refreshDevices();
}

CaptureAVFoundation::~CaptureAVFoundation() {
  stopCapture();
  capture_settings->deleteAllChildren();
}

void CaptureAVFoundation::refreshDevices() {
  v_device->resetToDefault();
  v_device->addItem("Default");

  NSArray<AVCaptureDevice *> *devices =
      [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
  for (AVCaptureDevice *device in devices) {
    std::string name = [device.localizedName UTF8String] ? [device.localizedName UTF8String] : "Unknown";
    v_device->addItem(name);
  }
}

bool CaptureAVFoundation::ensureDeviceSelection() {
  NSString *requested = [NSString stringWithUTF8String:v_device->getString().c_str()];
  AVCaptureDevice *device = nil;
  if ([requested isEqualToString:@"Default"]) {
    device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
  } else {
    for (AVCaptureDevice *candidate in
         [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo]) {
      if ([candidate.localizedName isEqualToString:requested]) {
        device = candidate;
        break;
      }
    }
  }

  if (!device) {
    fprintf(stderr, "[AVFoundation] No camera device available\n");
    return false;
  }

  if (!applyFormatPreferences(device)) {
    fprintf(stderr, "[AVFoundation] Failed to configure device format\n");
    return false;
  }

  NSError *error = nil;
  device_input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&error];
  if (!device_input || error) {
    fprintf(stderr, "[AVFoundation] Failed to create device input: %s\n",
            error.localizedDescription.UTF8String);
    return false;
  }

  if (diagnosticsEnabled()) {
    std::string deviceName = device.localizedName ? [device.localizedName UTF8String] : "Unknown";
    std::ostringstream oss;
    oss << "ensureDeviceSelection -> device='" << deviceName << "' requested='" << v_device->getString() << "'";
    logDiagnostic(oss.str());
  }

  return true;
}

bool CaptureAVFoundation::applyFormatPreferences(AVCaptureDevice *device) {
  NSError *error = nil;
  if (![device lockForConfiguration:&error]) {
    fprintf(stderr, "[AVFoundation] lockForConfiguration failed: %s\n",
            error.localizedDescription.UTF8String);
    return false;
  }

  const int desiredWidth = std::max(160, v_width->getInt());
  const int desiredHeight = std::max(120, v_height->getInt());
  const double desiredFps = std::max(1, v_fps->getInt());

  AVCaptureDeviceFormat *selectedFormat = device.activeFormat;
  double bestScore = std::numeric_limits<double>::max();

  for (AVCaptureDeviceFormat *format in device.formats) {
    CMFormatDescriptionRef desc = format.formatDescription;
    CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(desc);
    double widthDiff = std::abs(dims.width - desiredWidth);
    double heightDiff = std::abs(dims.height - desiredHeight);

    double score = widthDiff + heightDiff;
    if (score < bestScore) {
      bestScore = score;
      selectedFormat = format;
    }
  }

  double actualFps = desiredFps;
  if (selectedFormat) {
    device.activeFormat = selectedFormat;

    // Determine the maximum supported frame rate for the selected format
    double maxSupportedFps = 0.0;
    for (AVFrameRateRange *range in selectedFormat.videoSupportedFrameRateRanges) {
      maxSupportedFps = std::max(maxSupportedFps, range.maxFrameRate);
    }

    // Clamp desired FPS to the maximum supported by the format
    actualFps = std::min(desiredFps, maxSupportedFps);
    if (actualFps <= 0.0) {
      actualFps = 1.0;
    }
  }

  [device unlockForConfiguration];

  if (diagnosticsEnabled()) {
    CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(device.activeFormat.formatDescription);
    std::ostringstream oss;
    oss << "applyFormatPreferences -> desired=" << desiredWidth << "x" << desiredHeight
        << "@" << desiredFps << "fps | selected=" << dims.width << "x" << dims.height
        << "@" << actualFps << "fps";
    logDiagnostic(oss.str());
  }
  return true;
}

bool CaptureAVFoundation::configureSession() {
  if (!ensureAuthorization()) {
    return false;
  }

  session = [[AVCaptureSession alloc] init];
  if (!session) {
    fprintf(stderr, "[AVFoundation] Failed to create session\n");
    return false;
  }

  if (!ensureDeviceSelection()) {
    return false;
  }

  if (![session canAddInput:device_input]) {
    fprintf(stderr, "[AVFoundation] Unable to add device input\n");
    return false;
  }
  [session addInput:device_input];

  video_output = [[AVCaptureVideoDataOutput alloc] init];
  video_output.videoSettings = @{(id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)};
  video_output.alwaysDiscardsLateVideoFrames = NO;

  if (![session canAddOutput:video_output]) {
    fprintf(stderr, "[AVFoundation] Unable to add video output\n");
    return false;
  }
  [session addOutput:video_output];

  capture_queue = dispatch_queue_create("org.ssl-vision.avfoundation", DISPATCH_QUEUE_SERIAL);
  delegate = [[CaptureAVFoundationFrameDelegate alloc] init];
  delegate.owner = this;
  [video_output setSampleBufferDelegate:delegate queue:capture_queue];

  AVCaptureConnection *connection = [video_output connectionWithMediaType:AVMediaTypeVideo];
  if (connection && connection.isVideoMirroringSupported) {
    connection.automaticallyAdjustsVideoMirroring = NO;
    connection.videoMirrored = v_mirror->getBool();
  }

  if (connection && connection.isVideoOrientationSupported) {
    connection.videoOrientation = AVCaptureVideoOrientationLandscapeRight;
  }

  double configuredFps = 0.0;
  if (connection) {
    // Clamp FPS to device's supported range for the active format
    double desiredFps = std::max(1, v_fps->getInt());
    double actualFps = desiredFps;

    AVCaptureDevice *device = device_input.device;
    if (device && device.activeFormat) {
      double maxSupportedFps = 0.0;
      for (AVFrameRateRange *range in device.activeFormat.videoSupportedFrameRateRanges) {
        maxSupportedFps = std::max(maxSupportedFps, range.maxFrameRate);
      }
      actualFps = std::min(desiredFps, maxSupportedFps);
      if (actualFps <= 0.0) {
        actualFps = 1.0;
      }
    }

    CMTime minFrameDuration = CMTimeMake(1, static_cast<int32_t>(actualFps));
    connection.videoMinFrameDuration = minFrameDuration;
    connection.videoMaxFrameDuration = minFrameDuration;
    configuredFps = actualFps;
  }

  if (diagnosticsEnabled()) {
    std::ostringstream oss;
    oss << "configureSession -> queue='" << (capture_queue ? "created" : "null")
        << "' configuredFps=" << configuredFps;
    logDiagnostic(oss.str());
  }

  return true;
}

void CaptureAVFoundation::teardownSession() {
  if (session) {
    [session stopRunning];
  }
  if (video_output) {
    [video_output setSampleBufferDelegate:nil queue:NULL];
    video_output = nil;
  }
  if (delegate) {
    delegate.owner = nullptr;
    [delegate release];
    delegate = nil;
  }
  if (device_input) {
    device_input = nil;
  }
  if (session) {
    [session release];
    session = nil;
  }
  if (capture_queue) {
    dispatch_release(capture_queue);
    capture_queue = nullptr;
  }
}

bool CaptureAVFoundation::startCapture() {
  QMutexLocker locker(&frame_mutex);
  if (is_capturing) {
    return true;
  }

  if (!configureSession()) {
    teardownSession();
    return false;
  }

  [session startRunning];
  is_capturing = true;
  frame_queue.clear();

  // Wait briefly for the first frame to arrive so we can fail fast
  // when capture permissions or device selection are not working.
  for (int i = 0; i < 10 && frame_queue.empty(); ++i) {
    frame_available.wait(&frame_mutex, 200);
  }
  if (frame_queue.empty()) {
    fprintf(stderr, "[AVFoundation] No frames received after start; stopping capture\n");
    is_capturing = false;
    locker.unlock();
    teardownSession();
    return false;
  }
  return true;
}

bool CaptureAVFoundation::stopCapture() {
  QMutexLocker locker(&frame_mutex);
  if (!is_capturing)
    return true;

  is_capturing = false;
  frame_queue.clear();
  frame_buffer.clear();
  frame_available.wakeAll();
  teardownSession();
  return true;
}

bool CaptureAVFoundation::ensureAuthorization() {
  AVAuthorizationStatus status =
      [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
  if (status == AVAuthorizationStatusAuthorized) {
    return true;
  }
  if (status == AVAuthorizationStatusDenied || status == AVAuthorizationStatusRestricted) {
    fprintf(stderr,
            "[AVFoundation] Camera access denied or restricted. Enable camera access for ssl-vision in System Settings.\n");
    return false;
  }
  if (status == AVAuthorizationStatusNotDetermined) {
    __block bool granted = false;
    dispatch_semaphore_t sema = dispatch_semaphore_create(0);
    dispatch_async(dispatch_get_main_queue(), ^{
      [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
                               completionHandler:^(BOOL didGrant) {
                                 granted = didGrant;
                                 dispatch_semaphore_signal(sema);
                               }];
    });
    dispatch_time_t timeout = dispatch_time(DISPATCH_TIME_NOW, (int64_t)(5 * NSEC_PER_SEC));
    dispatch_semaphore_wait(sema, timeout);
    if (!granted) {
      fprintf(stderr,
              "[AVFoundation] Camera access not granted. Enable camera access for ssl-vision in System Settings.\n");
      return false;
    }
    return true;
  }

  fprintf(stderr, "[AVFoundation] Unable to determine camera authorization status.\n");
  return false;
}

RawImage CaptureAVFoundation::getFrame() {
  QMutexLocker locker(&frame_mutex);
  for (int i = 0; i < 3 && frame_queue.empty(); ++i) {
    frame_available.wait(&frame_mutex, kFrameWaitTimeoutMs);
  }

  if (frame_queue.empty()) {
    if (frame_buffer.getWidth() > 0 && frame_buffer.getHeight() > 0 && frame_buffer.getData() != nullptr) {
      if (diagnosticsEnabled()) {
        logDiagnostic("frame_queue empty after wait; reusing last frame");
      }
      frame_buffer.setTime(GetTimeSec());
      return frame_buffer;
    }
    if (diagnosticsEnabled()) {
      logDiagnostic("frame_queue empty after wait; returning zero-sized frame");
    }
    frame_buffer.setColorFormat(COLOR_RGB8);
    frame_buffer.clear();
    frame_buffer.setWidth(0);
    frame_buffer.setHeight(0);
    frame_buffer.setData(nullptr);
    return frame_buffer;
  }

  CapturedFrame frame = frame_queue.front();
  frame_queue.pop_front();
  if (diagnosticsEnabled()) {
    std::ostringstream oss;
    oss << "Dequeued frame " << frame.width << "x" << frame.height
        << " | queue_size=" << frame_queue.size();
    logDiagnostic(oss.str());
  }
  locker.unlock();

  if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty()) {
    if (diagnosticsEnabled()) {
      logDiagnostic("Dequeued invalid frame (zero dimensions or empty pixels)");
    }
    frame_buffer.setColorFormat(COLOR_RGB8);
    frame_buffer.clear();
    frame_buffer.setWidth(0);
    frame_buffer.setHeight(0);
    frame_buffer.setData(nullptr);
    return frame_buffer;
  }

  frame_buffer.ensure_allocation(COLOR_RGB8, frame.width, frame.height);
  std::memcpy(frame_buffer.getData(), frame.pixels.data(), frame_buffer.getNumBytes());
  frame_buffer.setTime(frame.timestamp);
  frame_buffer.setTimeCam(frame.timestamp_cam);
  frame_buffer.setWidth(frame.width);
  frame_buffer.setHeight(frame.height);
  frame_buffer.setColorFormat(COLOR_RGB8);

  if (diagnosticsEnabled()) {
    std::ostringstream oss;
    oss << "Returning frame " << frame.width << "x" << frame.height
        << " timestamp=" << frame.timestamp_cam;
    logDiagnostic(oss.str());
  }

  return frame_buffer;
}

void CaptureAVFoundation::releaseFrame() {
  // No buffering past frame_queue logic
}

void CaptureAVFoundation::readAllParameterValues() {
  refreshDevices();
}

void CaptureAVFoundation::handleSampleBuffer(CMSampleBufferRef sampleBuffer) {
  CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
  if (!imageBuffer) {
    return;
  }

  CVPixelBufferRetain(imageBuffer);
  CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

  size_t width = CVPixelBufferGetWidth(imageBuffer);
  size_t height = CVPixelBufferGetHeight(imageBuffer);
  size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
  unsigned char *baseAddress = static_cast<unsigned char *>(CVPixelBufferGetBaseAddress(imageBuffer));

  if (diagnosticsEnabled()) {
    std::ostringstream oss;
    oss << "handleSampleBuffer -> " << width << "x" << height
        << " bytesPerRow=" << bytesPerRow;
    logDiagnostic(oss.str());
  }

  // Get the actual timestamp from the sample buffer for accurate frame rate
  CMTime sampleTime = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
  double timestampCam = (CMTimeGetSeconds(sampleTime) * 60000.0);

  // Convert BGRA -> RGB
  CapturedFrame frame;
  frame.width = static_cast<int>(width);
  frame.height = static_cast<int>(height);
  frame.timestamp = GetTimeSec();
  frame.timestamp_cam = timestampCam;
  const size_t expectedBytes = width * height * 3;
  frame.pixels.resize(expectedBytes);

  vImage_Buffer src = { (void*)baseAddress, static_cast<vImagePixelCount>(height), static_cast<vImagePixelCount>(width), bytesPerRow };
  vImage_Buffer dst = { frame.pixels.data(), static_cast<vImagePixelCount>(height), static_cast<vImagePixelCount>(width), static_cast<vImagePixelCount>(width * 3) };
  vImage_Error convertStatus = vImageConvert_BGRA8888toRGB888(&src, &dst, kvImageNoFlags);
  if (convertStatus != kvImageNoError) {
    fprintf(stderr, "[AVFoundation] vImageConvert_BGRA8888toRGB888 failed: %ld\n", convertStatus);
    frame.pixels.clear();
    CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
    CVPixelBufferRelease(imageBuffer);
    return;
  }

  CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
  CVPixelBufferRelease(imageBuffer);

  if (frame.pixels.empty()) {
    if (diagnosticsEnabled()) {
      logDiagnostic("Conversion produced empty pixel buffer; dropping frame");
    }
    return;
  }

  {
    QMutexLocker locker(&frame_mutex);
    if (frame_queue.size() >= kMaxBufferedFrames) {
      if (diagnosticsEnabled()) {
        logDiagnostic("Frame queue full; dropping oldest frame");
      }
      frame_queue.pop_front();
    }
    frame_queue.push_back(std::move(frame));
    if (diagnosticsEnabled()) {
      std::ostringstream oss;
      oss << "Queued frame | queue_size=" << frame_queue.size();
      logDiagnostic(oss.str());
    }
  }

  frame_available.wakeOne();
}

bool CaptureAVFoundation::diagnosticsEnabled() const {
  return v_diagnostics && v_diagnostics->getBool();
}

void CaptureAVFoundation::logDiagnostic(const std::string &message) const {
  if (diagnosticsEnabled()) {
    std::cout << "[AVFoundation][Diag] " << message << std::endl;
  }
}

#endif // __APPLE__
