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
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <QString>

#include "colors.h"
#include "rawimage.h"

namespace {
constexpr double kDefaultFPS = 60.0;
constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;
constexpr size_t kMaxBufferedFrames = 2;
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

  capture_settings->addChild(v_width);
  capture_settings->addChild(v_height);
  capture_settings->addChild(v_fps);
  capture_settings->addChild(v_mirror);

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

    bool supportsFps = false;
    for (AVFrameRateRange *range in format.videoSupportedFrameRateRanges) {
      if (desiredFps >= range.minFrameRate && desiredFps <= range.maxFrameRate) {
        supportsFps = true;
        break;
      }
    }
    if (!supportsFps) {
      continue;
    }

    double score = widthDiff + heightDiff;
    if (score < bestScore) {
      bestScore = score;
      selectedFormat = format;
    }
  }

  if (selectedFormat) {
    device.activeFormat = selectedFormat;
    CMTime desiredDuration = CMTimeMake(1, static_cast<int32_t>(std::max(1, static_cast<int>(desiredFps))));
    device.activeVideoMinFrameDuration = desiredDuration;
    device.activeVideoMaxFrameDuration = desiredDuration;
  }

  [device unlockForConfiguration];
  return true;
}

bool CaptureAVFoundation::configureSession() {
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
  video_output.alwaysDiscardsLateVideoFrames = YES;

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

  CMTime minFrameDuration = CMTimeMake(1, std::max(1, v_fps->getInt()));
  if (connection) {
    connection.videoMinFrameDuration = minFrameDuration;
    connection.videoMaxFrameDuration = minFrameDuration;
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

RawImage CaptureAVFoundation::getFrame() {
  QMutexLocker locker(&frame_mutex);
  if (frame_queue.empty()) {
    frame_available.wait(&frame_mutex, 50);
  }

  if (frame_queue.empty()) {
    frame_buffer.clear();
    return frame_buffer;
  }

  CapturedFrame frame = frame_queue.front();
  frame_queue.pop_front();
  locker.unlock();

  frame_buffer.ensure_allocation(COLOR_RGB8, frame.width, frame.height);
  std::memcpy(frame_buffer.getData(), frame.pixels.data(), frame_buffer.getNumBytes());
  frame_buffer.setTime(frame.timestamp);
  frame_buffer.setTimeCam(frame.timestamp_cam);
  frame_buffer.setWidth(frame.width);
  frame_buffer.setHeight(frame.height);
  frame_buffer.setColorFormat(COLOR_RGB8);
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

  CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

  size_t width = CVPixelBufferGetWidth(imageBuffer);
  size_t height = CVPixelBufferGetHeight(imageBuffer);
  size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
  unsigned char *baseAddress = static_cast<unsigned char *>(CVPixelBufferGetBaseAddress(imageBuffer));

  // Convert BGRA -> RGB
  CapturedFrame frame;
  frame.width = static_cast<int>(width);
  frame.height = static_cast<int>(height);
  frame.timestamp = GetTimeSec();
  frame.timestamp_cam = CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer));
  frame.pixels.resize(width * height * 3);

  for (size_t y = 0; y < height; ++y) {
    const unsigned char *src = baseAddress + y * bytesPerRow;
    unsigned char *dst = frame.pixels.data() + y * width * 3;
    for (size_t x = 0; x < width; ++x) {
      // BGRA order
      dst[0] = src[2]; // R
      dst[1] = src[1]; // G
      dst[2] = src[0]; // B
      src += 4;
      dst += 3;
    }
  }

  CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

  QMutexLocker locker(&frame_mutex);
  if (frame_queue.size() >= kMaxBufferedFrames) {
    frame_queue.pop_front();
  }
  frame_queue.push_back(std::move(frame));
  frame_available.wakeOne();
}

#endif // __APPLE__
