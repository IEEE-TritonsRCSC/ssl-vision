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
  \file    plugin_cameracalib.cpp
  \brief   C++ Implementation: plugin_cameracalib
  \author  Tim Laue, 2009
*/
//========================================================================

#include "plugin_cameracalib.h"
#include "plugin_visualize.h"
#include "conversions.h"
#include "sobel.h"
#include "field_default_constants.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <QTabWidget>
#include <QStackedWidget>
#include <iostream>

using std::swap;

struct FieldLineSelectionState {
  bool isSelecting;
  QPointF startPoint;
  QPointF endPoint;
  PluginCameraCalibration::FieldLineType lineType;
  
  FieldLineSelectionState()
    : isSelecting(false), lineType(PluginCameraCalibration::LINE_NONE) {}
};

PluginCameraCalibration::PluginCameraCalibration(
    FrameBuffer* _buffer, CameraParameters& camera_params,
    RoboCupField& _field) :
    VisionPlugin(_buffer), camera_parameters(camera_params),
    field(_field),
    ccw(nullptr), grey_image(nullptr), rgb_image(nullptr), drag_x(nullptr),
    drag_y(nullptr), calib_drag_x(nullptr), calib_drag_y(nullptr),
    isSelectingFieldLine(false), currentLineType(LINE_NONE),
    isCustomCalibrationMode(false), customClickCount(0),
    useCustomCalibrationCameraModel(true) {
  video_width=video_height=0;
  settings=new VarList("Camera Calibrator");
  settings->addChild(camera_settings = new VarList("Camera Parameters"));
  camera_params.addSettingsToList(*camera_settings);
  settings->addChild(calibration_settings =
      new VarList("Calibration Parameters"));
  camera_parameters.additional_calibration_information->addSettingsToList(
      *calibration_settings);
}

PluginCameraCalibration::~PluginCameraCalibration() {
  delete camera_settings;
  delete calibration_settings;
  if(grey_image)
    delete grey_image;
  if(rgb_image)
    delete rgb_image;
}

void PluginCameraCalibration::detectEdges(FrameData* data) {
  double point_separation(camera_parameters.additional_calibration_information->
      pointSeparation->getDouble());
  // Reset list:
  camera_parameters.calibrationSegments.clear();
  // Get a greyscale image:
  if(grey_image == 0) {
    grey_image = new greyImage(data->video.getWidth(),data->video.getHeight());
    rgb_image = new rgbImage(data->video.getWidth(),data->video.getHeight());
  }
  if (data->video.getColorFormat()==COLOR_YUV422_UYVY) {
    Conversions::uyvy2rgb(
        data->video.getData(),
        reinterpret_cast<unsigned char*>(rgb_image->getData()),
        data->video.getWidth(),data->video.getHeight());
    Images::convert(*rgb_image, *grey_image);
  } else if (data->video.getColorFormat()==COLOR_RGB8) {
    Images::convert(data->video, *grey_image);
  } else {
    fprintf(stderr, "ColorThresholding needs YUV422 or RGB8 as input image, "
            "but found: %s\n",
            Colors::colorFormatToString(data->video.getColorFormat()).c_str());
    return;
  }

  field.field_markings_mutex.lockForRead();
  for (size_t i = 0; i < field.field_lines.size(); ++i) {
    const FieldLine& line = *(field.field_lines[i]);
    const GVector::vector3d<double> p1(
        line.p1_x->getDouble(), line.p1_y->getDouble(), 0.0);
    const GVector::vector3d<double> p2(
        line.p2_x->getDouble(), line.p2_y->getDouble(), 0.0);
    detectEdgesOnSingleLine(
        p1, p2, line.thickness->getDouble(), point_separation);
  }

  for (size_t i = 0; i < field.field_arcs.size(); ++i) {
    const FieldCircularArc& arc = *(field.field_arcs[i]);
    const GVector::vector3d<double> center(
        arc.center_x->getDouble(), arc.center_y->getDouble(), 0.0);
    detectEdgesOnSingleArc(center, arc.radius->getDouble(), arc.a1->getDouble(),
        arc.a2->getDouble(), arc.thickness->getDouble(), point_separation);
  }

  if (camera_parameters.use_opencv_model->getBool()) {
    camera_parameters.detectCalibrationCorners();
  }

  field.field_markings_mutex.unlock();
}

ProcessResult PluginCameraCalibration::process(
    FrameData* data, RenderOptions* options) {
  video_width=data->video.getWidth();
  video_height=data->video.getHeight();
  if(camera_parameters.additional_calibration_information->imageWidth->getInt() != video_width ||
     camera_parameters.additional_calibration_information->imageHeight->getInt() != video_height){
      camera_parameters.additional_calibration_information->imageWidth->setInt(video_width);
      camera_parameters.additional_calibration_information->imageHeight->setInt(video_height);
  }
  (void)options;

  if(ccw) {
    if(ccw->getDetectEdges()) {
      detectEdges(data);
      // detectEdges2(data);
      ccw->resetDetectEdges();
    }
    ccw->set_slider_from_vars();
  }
  return ProcessingOk;
}

VarList * PluginCameraCalibration::getSettings() {
  return settings;
}

string PluginCameraCalibration::getName() {
  return "Camera Calibration";
}

QWidget * PluginCameraCalibration::getControlWidget() {
  if (ccw==0)
    ccw = new CameraCalibrationWidget(camera_parameters);
  ccw->setPlugin(this);

  return (QWidget *)ccw;
}

void PluginCameraCalibration::sanitizeSobel(
    greyImage* img, GVector::vector2d<double>& val, int sobel_border) {
  val.x = bound<double>(val.x, sobel_border, img->getWidth() - sobel_border);
  val.y = bound<double>(val.y, sobel_border, img->getHeight() - sobel_border);
}

void PluginCameraCalibration::detectEdgesOnSingleLine(
    const GVector::vector3d<double>& p1,
    const GVector::vector3d<double>& p2,
    double thickness, double point_separation) {
  static const int kSobelThreshold = 20;
  const double search_distance =
      camera_parameters.additional_calibration_information->
      line_search_corridor_width->getDouble() / 2.0;
  const double image_boundary =
      camera_parameters.additional_calibration_information->
      image_boundary->getDouble();
  const double max_feature_distance =
      camera_parameters.additional_calibration_information->
      max_feature_distance->getDouble();
  CameraParameters::CalibrationData calibration_data;
  calibration_data.straightLine = true;
  calibration_data.p1 = p1;
  calibration_data.p2 = p2;
  const double line_length = (p2 - p1).length();
  const GVector::vector3d<double> line_dir = (p2 - p1) / line_length;
  const GVector::vector3d<double> line_perp(-line_dir.y, line_dir.x, 0.0);
  const int num_points = floor(line_length / point_separation - 1.0);
  for (int i = 1; i <= num_points; ++i) {
    const double alpha =
        1.0 - static_cast<double>(i) / (static_cast<double>(num_points) + 1.0);
    const GVector::vector3d<double> p_world = alpha * p1 + (1.0 - alpha) * p2;
    GVector::vector2d<double> p_image(0.0, 0.0);
        camera_parameters.field2image(p_world, p_image);
    if (p_image.x < image_boundary ||
        p_image.x > grey_image->getWidth() - image_boundary ||
        p_image.y < image_boundary ||
        p_image.y > grey_image->getHeight() - image_boundary) {
      // This edge feature is outside the image boundary, so ignore it since
      // the edge detection might not be accurate.
      continue;
    }
    if ((p_world - camera_parameters.getWorldLocation()).length() >
       max_feature_distance) {
      // This edge feature too far away from the camera, so ignore it since the
      // line feature is likely to be too small to detect accurately.
      continue;
    }
    const GVector::vector3d<double> start_world =
        p_world + line_perp * (0.5 * thickness + search_distance);
    const GVector::vector3d<double> end_world =
        p_world - line_perp * (0.5 * thickness + search_distance);
    GVector::vector2d<double> start_image(0.0, 0.0), end_image(0.0, 0.0);
    camera_parameters.field2image(start_world, start_image);
    camera_parameters.field2image(end_world, end_image);
    sanitizeSobel(grey_image, start_image);
    sanitizeSobel(grey_image, end_image);

    GVector::vector2d<double> point_image;
    bool center_found = false;
    Sobel::centerOfLine(
        *grey_image, start_image.x, end_image.x, start_image.y,
        end_image.y, point_image, center_found, kSobelThreshold);
    CameraParameters::CalibrationDataPoint point(point_image, center_found);
    calibration_data.points.push_back(point);
    calibration_data.alphas.push_back(alpha);
  }
  if (calibration_data.points.size() > 2) {
    // Need at least two points to uniquely define a line segment.
    camera_parameters.calibrationSegments.push_back(calibration_data);
  }
}

void PluginCameraCalibration::detectEdgesOnSingleArc(
    const GVector::vector3d<double>& center, double radius, double theta1,
    double theta2, double thickness, double point_separation) {
  static const int kSobelThreshold = 20;
  const double search_distance =
      camera_parameters.additional_calibration_information->
      line_search_corridor_width->getDouble() / 2.0;
  const double image_boundary =
      camera_parameters.additional_calibration_information->
      image_boundary->getDouble();
  const double max_feature_distance =
      camera_parameters.additional_calibration_information->
      max_feature_distance->getDouble();
  CameraParameters::CalibrationData calibration_data;
  calibration_data.straightLine = false;
  calibration_data.radius = radius;
  calibration_data.center = center;
  calibration_data.theta1 = theta1;
  calibration_data.theta2 = theta2;
  const int num_points =
      floor((theta2 - theta1) * radius / point_separation - 1.0);
  for (int i = 1; i <= num_points; ++i) {
    const double alpha =
        1.0 - static_cast<double>(i) / (static_cast<double>(num_points) + 1.0);
    const double theta = alpha * theta1 + (1.0 - alpha) * theta2;
    const GVector::vector3d<double> radius_vector(cos(theta), sin(theta), 0.0);
    const GVector::vector3d<double> p_world = center + radius * radius_vector;
    GVector::vector2d<double> p_image(0.0, 0.0);
    camera_parameters.field2image(p_world, p_image);
    if (p_image.x < image_boundary ||
        p_image.x > grey_image->getWidth() - image_boundary ||
        p_image.y < image_boundary ||
        p_image.y > grey_image->getHeight() - image_boundary) {
      // This edge feature is outside the image boundary, so ignore it since
      // the edge detection might not be accurate.
      continue;
    }
    if ((p_world - camera_parameters.getWorldLocation()).length() >
       max_feature_distance) {
      // This edge feature too far away from the camera, so ignore it since the
      // line feature is likely to be too small to detect accurately.
      continue;
    }
    const GVector::vector3d<double> start_world =
        center + (radius - 0.5 * thickness - search_distance) * radius_vector;
    const GVector::vector3d<double> end_world =
        center + (radius + 0.5 * thickness + search_distance) * radius_vector;
    GVector::vector2d<double> start_image(0.0, 0.0), end_image(0.0, 0.0);
    camera_parameters.field2image(start_world, start_image);
    camera_parameters.field2image(end_world, end_image);
    sanitizeSobel(grey_image, start_image);
    sanitizeSobel(grey_image, end_image);

    GVector::vector2d<double> point_image;
    bool center_found = false;
    Sobel::centerOfLine(
        *grey_image, start_image.x, end_image.x, start_image.y,
        end_image.y, point_image, center_found, kSobelThreshold);
    CameraParameters::CalibrationDataPoint point(point_image, center_found);
    calibration_data.points.push_back(point);
    calibration_data.alphas.push_back(alpha);
  }
  if (calibration_data.points.size() > 3) {
    // Need at least three points to uniquely define a circular arc.
    camera_parameters.calibrationSegments.push_back(calibration_data);
  }
}

void PluginCameraCalibration::keyPressEvent ( QKeyEvent * event )
{
  (void) event;
}

void PluginCameraCalibration::mousePressEvent ( QMouseEvent * event, pixelloc loc )
{
  std::cout << "[PluginCameraCalibration] mousePressEvent received, buttons=" << event->buttons()
            << ", loc=(" << loc.x << "," << loc.y << ")" << std::endl;
  std::cout << "[PluginCameraCalibration] Mode states: isCustomCalibrationMode=" << isCustomCalibrationMode
            << ", isSelectingFieldLine=" << isSelectingFieldLine << ", currentLineType=" << currentLineType << std::endl;
  (void)event; // Suppress unused parameter warning
  if ((event->buttons() & Qt::LeftButton)!=0) {
    // Custom calibration mode takes precedence
    if (isCustomCalibrationMode) {
      std::cout << "[PluginCameraCalibration] Custom calibration mode active, accepting event" << std::endl;
      int x = loc.x;
      int y = loc.y;
      if (x < 0) x = 0;
      if (y < 0) y = 0;
      if (video_width > 0 && x >= video_width) x = video_width - 1;
      if (video_height > 0 && y >= video_height) y = video_height - 1;
      addCustomCalibrationPoint(QPointF(x, y));
      event->accept();
      return;
    }

    // Standard interactive calibration
    if (isSelectingFieldLine && currentLineType != LINE_NONE) {
      std::cout << "[PluginCameraCalibration] Interactive calibration mode active, accepting event" << std::endl;
      selectionStartPoint = QPointF(loc.x, loc.y);
      selectionEndPoint = selectionStartPoint;
      event->accept();
      return;
    }

    // Check for drag points regardless of tab visibility
    if (ccw) {
      std::cout << "[PluginCameraCalibration] ccw widget exists, checking for drag points..." << std::endl;
      double drag_threshold = 20; //in px
      drag_x = nullptr;
      drag_y = nullptr;
      calib_drag_x = nullptr;
      calib_drag_y = nullptr;
      for (int i = 0; i < camera_parameters.extrinsic_parameters->getCalibrationPointSize(); i++) {
        auto point_x = camera_parameters.extrinsic_parameters->getCalibImageValueX(i);
        auto point_y = camera_parameters.extrinsic_parameters->getCalibImageValueY(i);
        double x_diff = point_x->getDouble() - loc.x;
        double y_diff = point_y->getDouble() - loc.y;
        if (sqrt(x_diff*x_diff + y_diff*y_diff) < drag_threshold) {
          std::cout << "[PluginCameraCalibration] Found calibration point at index " << i << ", accepting event" << std::endl;
          calib_drag_x = point_x;
          calib_drag_y = point_y;
          event->accept();
          return;
        }
      }
      for (int i = 0; i < CameraParameters::AdditionalCalibrationInformation::kNumControlPoints; ++i) {
        const double x_diff =
            camera_parameters.additional_calibration_information->
                control_point_image_xs[i]->getDouble() - loc.x;
        const double y_diff =
            camera_parameters.additional_calibration_information->
                control_point_image_ys[i]->getDouble() - loc.y;
        if (sqrt(x_diff*x_diff + y_diff*y_diff) < drag_threshold) {
          std::cout << "[PluginCameraCalibration] Found control point at index " << i << ", accepting event" << std::endl;
          drag_x = camera_parameters.additional_calibration_information->
              control_point_image_xs[i];
          drag_y = camera_parameters.additional_calibration_information->
              control_point_image_ys[i];
          event->accept();
          return;
        }
      }
      std::cout << "[PluginCameraCalibration] No drag points found near click location" << std::endl;
    } else {
      std::cout << "[PluginCameraCalibration] ccw widget is NULL" << std::endl;
    }
  }
  std::cout << "[PluginCameraCalibration] Ignoring event" << std::endl;
  event->ignore();
}

void PluginCameraCalibration::mouseReleaseEvent ( QMouseEvent * event, pixelloc loc )
{
  (void)loc;
  // Standard interactive calibration
  if (isSelectingFieldLine && currentLineType != LINE_NONE) {
    applyFieldLineCalibration(selectionStartPoint, selectionEndPoint, currentLineType);
    isSelectingFieldLine = false;
    selectionStartPoint = QPointF();
    selectionEndPoint = QPointF();
    event->accept();
    return;
  }
  
  // Handle drag release regardless of tab visibility
  if (ccw && (drag_x || drag_y || calib_drag_x || calib_drag_y)) {
    drag_x = nullptr;
    drag_y = nullptr;
    calib_drag_x = nullptr;
    calib_drag_y = nullptr;
    event->accept();
    return;
  }
  event->ignore();
}

void PluginCameraCalibration::mouseMoveEvent ( QMouseEvent * event, pixelloc loc )
{
  std::cout << "[PluginCameraCalibration] mouseMoveEvent received, buttons=" << event->buttons()
            << ", loc=(" << loc.x << "," << loc.y << ")" << std::endl;
  std::cout << "[PluginCameraCalibration] Drag state: drag_x=" << (drag_x != nullptr)
            << ", drag_y=" << (drag_y != nullptr)
            << ", calib_drag_x=" << (calib_drag_x != nullptr)
            << ", calib_drag_y=" << (calib_drag_y != nullptr) << std::endl;
  if ((event->buttons() & Qt::LeftButton)!=0)
  {
    // Standard interactive calibration
    if (isSelectingFieldLine) {
      std::cout << "[PluginCameraCalibration] Interactive calibration drag, accepting" << std::endl;
      if (loc.x < 0) loc.x=0;
      if (loc.y < 0) loc.y=0;
      if (video_width > 0 && loc.x >= video_width) loc.x=video_width-1;
      if (video_height > 0 && loc.y >= video_height) loc.y=video_height-1;
      selectionEndPoint = QPointF(loc.x, loc.y);
      event->accept();
      return;
    }

    // Handle drag move regardless of tab visibility
    if (ccw && (drag_x || drag_y || calib_drag_x || calib_drag_y)) {
      std::cout << "[PluginCameraCalibration] Point dragging, updating coordinates" << std::endl;
      if (loc.x < 0) loc.x=0;
      if (loc.y < 0) loc.y=0;
      if (video_width > 0 && loc.x >= video_width) loc.x=video_width-1;
      if (video_height > 0 && loc.y >= video_height) loc.y=video_height-1;
      if (drag_x != nullptr && drag_y != nullptr) {
        drag_x->setDouble(loc.x);
        drag_y->setDouble(loc.y);
        event->accept();
      } else if (calib_drag_x != nullptr && calib_drag_y != nullptr) {
        calib_drag_x->setDouble(loc.x);
        calib_drag_y->setDouble(loc.y);
        event->accept();
      }
      return;
    }
  }
  std::cout << "[PluginCameraCalibration] Ignoring mouseMoveEvent" << std::endl;
  event->ignore();
}

void PluginCameraCalibration::setInteractiveCalibrationMode(bool enabled) {
  std::cout << "[PluginCameraCalibration] setInteractiveCalibrationMode called with enabled=" << enabled << std::endl;
  isSelectingFieldLine = enabled;
  if (!enabled) {
    currentLineType = LINE_NONE;
    selectionStartPoint = QPointF();
    selectionEndPoint = QPointF();
  }
}

void PluginCameraCalibration::setCurrentLineType(FieldLineType type) {
  currentLineType = type;
}

void PluginCameraCalibration::applyFieldLineCalibration(const QPointF& startPt, const QPointF& endPt, FieldLineType lineType) {
  GVector::vector2d<double> startImage(startPt.x(), startPt.y());
  GVector::vector2d<double> endImage(endPt.x(), endPt.y());

  GVector::vector3d<double> startField, endField;
  camera_parameters.image2field(startField, startImage, 0.0);
  camera_parameters.image2field(endField, endImage, 0.0);

  auto aci = camera_parameters.additional_calibration_information;

  switch (lineType) {
    case LINE_TOP:
      aci->control_point_field_xs[0]->setDouble(startField.x);
      aci->control_point_field_ys[0]->setDouble(startField.y);
      aci->control_point_field_xs[1]->setDouble(endField.x);
      aci->control_point_field_ys[1]->setDouble(endField.y);
      aci->control_point_image_xs[0]->setDouble(startPt.x());
      aci->control_point_image_ys[0]->setDouble(startPt.y());
      aci->control_point_image_xs[1]->setDouble(endPt.x());
      aci->control_point_image_ys[1]->setDouble(endPt.y());
      break;
    case LINE_BOTTOM:
      aci->control_point_field_xs[2]->setDouble(startField.x);
      aci->control_point_field_ys[2]->setDouble(startField.y);
      aci->control_point_field_xs[3]->setDouble(endField.x);
      aci->control_point_field_ys[3]->setDouble(endField.y);
      aci->control_point_image_xs[2]->setDouble(startPt.x());
      aci->control_point_image_ys[2]->setDouble(startPt.y());
      aci->control_point_image_xs[3]->setDouble(endPt.x());
      aci->control_point_image_ys[3]->setDouble(endPt.y());
      break;
    case LINE_LEFT:
      aci->control_point_field_xs[0]->setDouble(startField.x);
      aci->control_point_field_ys[0]->setDouble(startField.y);
      aci->control_point_field_xs[2]->setDouble(endField.x);
      aci->control_point_field_ys[2]->setDouble(endField.y);
      aci->control_point_image_xs[0]->setDouble(startPt.x());
      aci->control_point_image_ys[0]->setDouble(startPt.y());
      aci->control_point_image_xs[2]->setDouble(endPt.x());
      aci->control_point_image_ys[2]->setDouble(endPt.y());
      break;
    case LINE_RIGHT:
      aci->control_point_field_xs[1]->setDouble(startField.x);
      aci->control_point_field_ys[1]->setDouble(startField.y);
      aci->control_point_field_xs[3]->setDouble(endField.x);
      aci->control_point_field_ys[3]->setDouble(endField.y);
      aci->control_point_image_xs[1]->setDouble(startPt.x());
      aci->control_point_image_ys[1]->setDouble(startPt.y());
      aci->control_point_image_xs[3]->setDouble(endPt.x());
      aci->control_point_image_ys[3]->setDouble(endPt.y());
      break;
    case LINE_TOP_GOAL:
      aci->control_point_field_xs[0]->setDouble(startField.x);
      aci->control_point_field_ys[0]->setDouble(startField.y);
      aci->control_point_image_xs[0]->setDouble(startPt.x());
      aci->control_point_image_ys[0]->setDouble(startPt.y());
      break;
    case LINE_BOTTOM_GOAL:
      aci->control_point_field_xs[1]->setDouble(startField.x);
      aci->control_point_field_ys[1]->setDouble(startField.y);
      aci->control_point_image_xs[1]->setDouble(startPt.x());
      aci->control_point_image_ys[1]->setDouble(startPt.y());
      break;
    default:
      break;
  }
}

void PluginCameraCalibration::setCustomCalibrationMode(bool enabled) {
  isCustomCalibrationMode = enabled;
  if (!enabled) {
    selectionStartPoint = QPointF();
    selectionEndPoint = QPointF();
  }
  if (ccw) {
    ccw->updateCustomCalibStatus();
  }
}

void PluginCameraCalibration::setCustomCalibrationUseCameraModel(bool enabled) {
  useCustomCalibrationCameraModel = enabled;
  if (customClickCount == kCustomCalibrationPointCount) {
    applyCustomCalibrationFromPoints();
  }
}

void PluginCameraCalibration::addCustomCalibrationPoint(const QPointF& point) {
  if (customClickCount >= kCustomCalibrationPointCount) {
    return;
  }
  customClickPoints[customClickCount] = point;
  ++customClickCount;
  if (ccw) {
    ccw->updateCustomCalibStatus();
  }
  if (customClickCount == kCustomCalibrationPointCount) {
    applyCustomCalibrationFromPoints();
  }
}

void PluginCameraCalibration::applyCustomCalibrationFromPoints() {
  if (customClickCount < kCustomCalibrationPointCount) {
    return;
  }

  struct PointInfo {
    QPointF image;
    GVector::vector3d<double> field;
  };

  std::vector<PointInfo> points;
  points.reserve(kCustomCalibrationPointCount);
  for (int i = 0; i < kCustomCalibrationPointCount; ++i) {
    PointInfo info;
    info.image = customClickPoints[i];
    points.push_back(info);
  }

  if (useCustomCalibrationCameraModel) {
    for (auto& point : points) {
      GVector::vector2d<double> image_point(point.image.x(), point.image.y());
      camera_parameters.image2field(point.field, image_point, 0.0);
    }
  }

  if (useCustomCalibrationCameraModel) {
    std::sort(points.begin(), points.end(),
              [](const PointInfo& a, const PointInfo& b) {
                return a.field.y > b.field.y;
              });
  } else {
    std::sort(points.begin(), points.end(),
              [](const PointInfo& a, const PointInfo& b) {
                return a.image.y() < b.image.y();
              });
  }

  std::vector<PointInfo> top(points.begin(), points.begin() + 2);
  std::vector<PointInfo> bottom(points.begin() + 2, points.end());

  if (useCustomCalibrationCameraModel) {
    std::sort(top.begin(), top.end(),
              [](const PointInfo& a, const PointInfo& b) {
                return a.field.x < b.field.x;
              });
    std::sort(bottom.begin(), bottom.end(),
              [](const PointInfo& a, const PointInfo& b) {
                return a.field.x < b.field.x;
              });
  } else {
    std::sort(top.begin(), top.end(),
              [](const PointInfo& a, const PointInfo& b) {
                return a.image.x() < b.image.x();
              });
    std::sort(bottom.begin(), bottom.end(),
              [](const PointInfo& a, const PointInfo& b) {
                return a.image.x() < b.image.x();
              });
  }

  PointInfo top_left = top[0];
  PointInfo top_right = top[1];
  PointInfo bottom_left = bottom[0];
  PointInfo bottom_right = bottom[1];

  if (useCustomCalibrationCameraModel) {
    GVector::vector3d<double> center =
        (top_left.field + top_right.field + bottom_left.field + bottom_right.field) / 4.0;
    top_left.field -= center;
    top_right.field -= center;
    bottom_left.field -= center;
    bottom_right.field -= center;
  } else {
    const double field_length_config = field.field_length->getDouble();
    const double field_width_config = field.field_width->getDouble();
    const double long_side = std::max(field_length_config, field_width_config);
    const double short_side = std::min(field_length_config, field_width_config);

    const double top_edge_len = std::hypot(
        top_right.image.x() - top_left.image.x(),
        top_right.image.y() - top_left.image.y());
    const double left_edge_len = std::hypot(
        bottom_left.image.x() - top_left.image.x(),
        bottom_left.image.y() - top_left.image.y());
    const bool top_is_long = top_edge_len >= left_edge_len;

    if (top_is_long) {
      top_left.field = GVector::vector3d<double>(-long_side * 0.5, short_side * 0.5, 0.0);
      top_right.field = GVector::vector3d<double>(long_side * 0.5, short_side * 0.5, 0.0);
      bottom_left.field = GVector::vector3d<double>(-long_side * 0.5, -short_side * 0.5, 0.0);
      bottom_right.field = GVector::vector3d<double>(long_side * 0.5, -short_side * 0.5, 0.0);
    } else {
      top_left.field = GVector::vector3d<double>(-short_side * 0.5, long_side * 0.5, 0.0);
      top_right.field = GVector::vector3d<double>(short_side * 0.5, long_side * 0.5, 0.0);
      bottom_left.field = GVector::vector3d<double>(-short_side * 0.5, -long_side * 0.5, 0.0);
      bottom_right.field = GVector::vector3d<double>(short_side * 0.5, -long_side * 0.5, 0.0);
    }
  }

  auto aci = camera_parameters.additional_calibration_information;
  aci->control_point_field_xs[0]->setDouble(top_left.field.x);
  aci->control_point_field_ys[0]->setDouble(top_left.field.y);
  aci->control_point_image_xs[0]->setDouble(top_left.image.x());
  aci->control_point_image_ys[0]->setDouble(top_left.image.y());

  aci->control_point_field_xs[1]->setDouble(top_right.field.x);
  aci->control_point_field_ys[1]->setDouble(top_right.field.y);
  aci->control_point_image_xs[1]->setDouble(top_right.image.x());
  aci->control_point_image_ys[1]->setDouble(top_right.image.y());

  aci->control_point_field_xs[2]->setDouble(bottom_left.field.x);
  aci->control_point_field_ys[2]->setDouble(bottom_left.field.y);
  aci->control_point_image_xs[2]->setDouble(bottom_left.image.x());
  aci->control_point_image_ys[2]->setDouble(bottom_left.image.y());

  aci->control_point_field_xs[3]->setDouble(bottom_right.field.x);
  aci->control_point_field_ys[3]->setDouble(bottom_right.field.y);
  aci->control_point_image_xs[3]->setDouble(bottom_right.image.x());
  aci->control_point_image_ys[3]->setDouble(bottom_right.image.y());

  customBoundaries[CUSTOM_TOP].defined = true;
  customBoundaries[CUSTOM_TOP].startField = top_left.field;
  customBoundaries[CUSTOM_TOP].endField = top_right.field;

  customBoundaries[CUSTOM_BOTTOM].defined = true;
  customBoundaries[CUSTOM_BOTTOM].startField = bottom_left.field;
  customBoundaries[CUSTOM_BOTTOM].endField = bottom_right.field;

  customBoundaries[CUSTOM_LEFT].defined = true;
  customBoundaries[CUSTOM_LEFT].startField = top_left.field;
  customBoundaries[CUSTOM_LEFT].endField = bottom_left.field;

  customBoundaries[CUSTOM_RIGHT].defined = true;
  customBoundaries[CUSTOM_RIGHT].startField = top_right.field;
  customBoundaries[CUSTOM_RIGHT].endField = bottom_right.field;

  updateFieldConfiguration();
  camera_parameters.do_calibration(CameraParameters::FOUR_POINT_INITIAL);
  if (ccw) {
    ccw->set_slider_from_vars();
  }
}

void PluginCameraCalibration::resetCustomCalibration() {
  customClickCount = 0;
  for (int i = 0; i < kCustomCalibrationPointCount; ++i) {
    customClickPoints[i] = QPointF();
  }
  resetCustomBoundaries();
  if (ccw) {
    ccw->updateCustomCalibStatus();
  }
}

void PluginCameraCalibration::resetCustomBoundaries() {
  for (int i = 0; i < 4; ++i) {
    customBoundaries[i].defined = false;
    customBoundaries[i].startField = GVector::vector3d<double>(0, 0, 0);
    customBoundaries[i].endField = GVector::vector3d<double>(0, 0, 0);
  }
}

int PluginCameraCalibration::getDefinedBoundaryCount() const {
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    if (customBoundaries[i].defined) {
      ++count;
    }
  }
  return count;
}

bool PluginCameraCalibration::areAllBoundariesDefined() const {
  return getDefinedBoundaryCount() == 4;
}

void PluginCameraCalibration::calculateCenterLine(double& p1_x, double& p1_y, double& p2_x, double& p2_y) {
  // Center line X is midpoint between left and right boundaries (average their X coordinates)
  double left_center_x = (customBoundaries[CUSTOM_LEFT].startField.x + customBoundaries[CUSTOM_LEFT].endField.x) / 2.0;
  double right_center_x = (customBoundaries[CUSTOM_RIGHT].startField.x + customBoundaries[CUSTOM_RIGHT].endField.x) / 2.0;
  double center_x = (left_center_x + right_center_x) / 2.0;
  
  // Y coordinates from top and bottom boundaries
  double top_y = (customBoundaries[CUSTOM_TOP].startField.y + customBoundaries[CUSTOM_TOP].endField.y) / 2.0;
  double bottom_y = (customBoundaries[CUSTOM_BOTTOM].startField.y + customBoundaries[CUSTOM_BOTTOM].endField.y) / 2.0;
  
  p1_x = center_x;
  p1_y = top_y;
  p2_x = center_x;
  p2_y = bottom_y;
}

void PluginCameraCalibration::calculateTopGoalLine(double& p1_x, double& p1_y, double& p2_x, double& p2_y) {
  // Top goal is at the midpoint of the top boundary
  double top_mid_x = (customBoundaries[CUSTOM_TOP].startField.x + customBoundaries[CUSTOM_TOP].endField.x) / 2.0;
  double top_mid_y = (customBoundaries[CUSTOM_TOP].startField.y + customBoundaries[CUSTOM_TOP].endField.y) / 2.0;
  
  // Calculate goal width (distance between left and right boundaries)
  double left_boundary_y = (customBoundaries[CUSTOM_LEFT].startField.y + customBoundaries[CUSTOM_LEFT].endField.y) / 2.0;
  double right_boundary_y = (customBoundaries[CUSTOM_RIGHT].startField.y + customBoundaries[CUSTOM_RIGHT].endField.y) / 2.0;
  double goal_half_width = (right_boundary_y - left_boundary_y) * 0.25; // 25% of field width for goal
  
  p1_x = top_mid_x;
  p1_y = top_mid_y - goal_half_width;
  p2_x = top_mid_x;
  p2_y = top_mid_y + goal_half_width;
}

void PluginCameraCalibration::calculateBottomGoalLine(double& p1_x, double& p1_y, double& p2_x, double& p2_y) {
  // Bottom goal is at the midpoint of the bottom boundary
  double bottom_mid_x = (customBoundaries[CUSTOM_BOTTOM].startField.x + customBoundaries[CUSTOM_BOTTOM].endField.x) / 2.0;
  double bottom_mid_y = (customBoundaries[CUSTOM_BOTTOM].startField.y + customBoundaries[CUSTOM_BOTTOM].endField.y) / 2.0;
  
  // Calculate goal width (distance between left and right boundaries)
  double left_boundary_y = (customBoundaries[CUSTOM_LEFT].startField.y + customBoundaries[CUSTOM_LEFT].endField.y) / 2.0;
  double right_boundary_y = (customBoundaries[CUSTOM_RIGHT].startField.y + customBoundaries[CUSTOM_RIGHT].endField.y) / 2.0;
  double goal_half_width = (right_boundary_y - left_boundary_y) * 0.25; // 25% of field width for goal
  
  p1_x = bottom_mid_x;
  p1_y = bottom_mid_y - goal_half_width;
  p2_x = bottom_mid_x;
  p2_y = bottom_mid_y + goal_half_width;
}

void PluginCameraCalibration::updateFieldConfiguration() {
  // Only update if all 4 boundaries are defined
  if (!areAllBoundariesDefined()) {
    return;
  }

  const double top_len =
      (customBoundaries[CUSTOM_TOP].endField -
       customBoundaries[CUSTOM_TOP].startField).length();
  const double bottom_len =
      (customBoundaries[CUSTOM_BOTTOM].endField -
       customBoundaries[CUSTOM_BOTTOM].startField).length();
  const double left_len =
      (customBoundaries[CUSTOM_LEFT].endField -
       customBoundaries[CUSTOM_LEFT].startField).length();
  const double right_len =
      (customBoundaries[CUSTOM_RIGHT].endField -
       customBoundaries[CUSTOM_RIGHT].startField).length();

  const double horizontal_len = (top_len + bottom_len) * 0.5;
  const double vertical_len = (left_len + right_len) * 0.5;

  if (horizontal_len <= 0.0 || vertical_len <= 0.0) {
    return;
  }

  const double field_length = std::max(horizontal_len, vertical_len);
  const double field_width = std::min(horizontal_len, vertical_len);

  const double length_ratio = field_length / FieldConstantsRoboCup2018B::kFieldLength;
  const double width_ratio = field_width / FieldConstantsRoboCup2018B::kFieldWidth;

  double rotation = 0.0;
  if (horizontal_len >= vertical_len) {
    GVector::vector3d<double> dir =
        customBoundaries[CUSTOM_TOP].endField -
        customBoundaries[CUSTOM_TOP].startField;
    rotation = std::atan2(dir.y, dir.x);
  } else {
    GVector::vector3d<double> dir =
        customBoundaries[CUSTOM_LEFT].endField -
        customBoundaries[CUSTOM_LEFT].startField;
    rotation = std::atan2(dir.y, dir.x);
  }

  field.field_length->setDouble(field_length);
  field.field_width->setDouble(field_width);
  field.field_rotation->setDouble(rotation);
  field.goal_width->setDouble(FieldConstantsRoboCup2018B::kGoalWidth * width_ratio);
  field.goal_depth->setDouble(FieldConstantsRoboCup2018B::kGoalDepth * length_ratio);
  field.goal_height->setDouble(FieldConstantsRoboCup2018B::kGoalHeight);
  field.boundary_width->setDouble(FieldConstantsRoboCup2018B::kBoundaryWidth * width_ratio);
  field.penalty_area_depth->setDouble(FieldConstantsRoboCup2018B::kPenaltyAreaDepth * length_ratio);
  field.penalty_area_width->setDouble(FieldConstantsRoboCup2018B::kPenaltyAreaWidth * width_ratio);
  field.center_circle_radius->setDouble(FieldConstantsRoboCup2018B::kCenterCircleRadius * width_ratio);
  field.goal_center_to_penalty_mark->setDouble(FieldConstantsRoboCup2018B::kGoalLineToPenaltyMark * length_ratio);

  field.rebuildFieldLinesAndArcs();

  emit field.calibrationChanged();
}

void PluginCameraCalibration::setVisualizePlugin(PluginVisualize* plugin) {
  if (ccw) {
    ccw->setVisualizePlugin(plugin);
  }
}
