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
  \file    plugin_cameracalib.h
  \brief   C++ Interface: plugin_cameracalib
  \author  Tim Laue, 2009
*/
//========================================================================
#ifndef PLUGIN_CAMERACALIB_H
#define PLUGIN_CAMERACALIB_H

#include "framedata.h"
#include "VarTypes.h"
#include "visionplugin.h"
#include "camera_calibration.h"
#include "field.h"
#include "image.h"
#include "cameracalibwidget.h"
#include <QPointF>

class PluginVisualize;

/**
*	@author Tim Laue <Tim.Laue@dfki.de>
*/
class PluginCameraCalibration : public VisionPlugin
{
public:
  enum FieldLineType {
    LINE_NONE = 0,
    LINE_TOP,
    LINE_BOTTOM,
    LINE_LEFT,
    LINE_RIGHT,
    LINE_TOP_GOAL,
    LINE_BOTTOM_GOAL
  };

  enum CustomFieldType {
    CUSTOM_TOP = 0,
    CUSTOM_BOTTOM,
    CUSTOM_LEFT,
    CUSTOM_RIGHT
  };

  struct CustomBoundaryLine {
    bool defined;
    GVector::vector3d<double> startField;
    GVector::vector3d<double> endField;
    
    CustomBoundaryLine() : defined(false), startField(0,0,0), endField(0,0,0) {}
  };

protected:
  VarList* settings;
  VarList* camera_settings;
  VarList* calibration_settings;
  CameraParameters& camera_parameters;
  RoboCupField& field;
  CameraCalibrationWidget * ccw;
  greyImage* grey_image;
  rgbImage* rgb_image;
  int video_width;
  int video_height;

  VarDouble* drag_x;
  VarDouble* drag_y;
  VarDouble* calib_drag_x;
  VarDouble* calib_drag_y;

  bool isSelectingFieldLine;
  QPointF selectionStartPoint;
  QPointF selectionEndPoint;
  FieldLineType currentLineType;

  // Custom calibration state
  bool isCustomCalibrationMode;
  CustomBoundaryLine customBoundaries[4];  // Top, Bottom, Left, Right
  static const int kCustomCalibrationPointCount = 4;
  QPointF customClickPoints[kCustomCalibrationPointCount];
  int customClickCount;
  bool useCustomCalibrationCameraModel;

  void sanitizeSobel(greyImage * img, GVector::vector2d<double> & val,int sobel_border=1);

  void detectEdges(FrameData * data);
  void detectEdgesOnSingleLine(const GVector::vector3d<double>& p1,
                                const GVector::vector3d<double>& p2,
                                double thickness, double point_separation);
  void detectEdgesOnSingleArc(const GVector::vector3d<double>& center,
                              double radius, double theta1, double theta2,
                              double thickness, double point_separation);

  void applyFieldLineCalibration(const QPointF& startPt, const QPointF& endPt, FieldLineType lineType);
  void applyCustomCalibrationFromPoints();
  void addCustomCalibrationPoint(const QPointF& point);

public:
  PluginCameraCalibration(
      FrameBuffer* _buffer, CameraParameters& camera_params,
      RoboCupField& _field);
  ~PluginCameraCalibration();
  virtual ProcessResult process(FrameData * data, RenderOptions * options);
  virtual VarList * getSettings();
  virtual std::string getName();
  virtual QWidget * getControlWidget();

  virtual void keyPressEvent ( QKeyEvent * event );
  virtual void mousePressEvent ( QMouseEvent * event, pixelloc loc );
  virtual void mouseReleaseEvent ( QMouseEvent * event, pixelloc loc );
  virtual void mouseMoveEvent ( QMouseEvent * event, pixelloc loc );

  void setInteractiveCalibrationMode(bool enabled);
  void setCurrentLineType(FieldLineType type);
  bool getIsSelectingFieldLine() const { return isSelectingFieldLine; }
  QPointF getSelectionStartPoint() const { return selectionStartPoint; }
  QPointF getSelectionEndPoint() const { return selectionEndPoint; }
  FieldLineType getCurrentLineType() const { return currentLineType; }

  // Accessors for custom calibration state (for visualization)
  bool getIsCustomCalibrationMode() const { return isCustomCalibrationMode; }
  int getCustomCalibrationPointCount() const { return customClickCount; }
  const QPointF* getCustomCalibrationPoints() const { return customClickPoints; }

  // Custom calibration methods
  void setCustomCalibrationMode(bool enabled);
  void setCustomCalibrationUseCameraModel(bool enabled);
  bool getCustomCalibrationUseCameraModel() const { return useCustomCalibrationCameraModel; }
  void resetCustomCalibration();
  void resetCustomBoundaries();
  int getDefinedBoundaryCount() const;
  bool areAllBoundariesDefined() const;
  
  // Field geometry calculation methods
  void updateFieldConfiguration();
  void calculateCenterLine(double& p1_x, double& p1_y, double& p2_x, double& p2_y);
  void calculateTopGoalLine(double& p1_x, double& p1_y, double& p2_x, double& p2_y);
  void calculateBottomGoalLine(double& p1_x, double& p1_y, double& p2_x, double& p2_y);
  
  // Accessor for custom boundaries (for visualization)
  const CustomBoundaryLine* getCustomBoundaries() const { return customBoundaries; }

  void setVisualizePlugin(PluginVisualize* plugin);

};

#endif //PLUGIN_CAMERACALIB_H
