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
  \file    mainwindow.cpp
  \brief   C++ Implementation: MainWindow
  \author  Stefan Zickler, (C) 2008
*/
//========================================================================

#include "mainwindow.h"
#include "../plugins/plugin_visualize.h"

MainWindow::MainWindow(bool start_capture, bool enforce_affinity, int num_cameras)
{

  affinity=0;
  if (enforce_affinity) affinity=new AffinityManager();
  //opt=new GetOpt();
  settings=0;
  setupUi((QMainWindow *)this);

  tree_view=new VarTreeView();
  tmodel=new VarTreeModel();

  splitter = new QSplitter(Qt::Horizontal,this);
  splitter->setOpaqueResize(true);
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(8);
  cam_tabs = new QTabWidget();

  root=new VarList("Vision System");

  save_settings_trigger = new VarTrigger("Save Settings", "Save Settings!");
  root->addChild(save_settings_trigger);

  opts=new RenderOptions();
  right_tab=0;

  QString stack_id="";

  //opt->addArgument("stack",&stack_id);
  //opt->parse();

  //load RoboCup SSL stack by default:
  multi_stack= new MultiStackRoboCupSSL(opts, num_cameras);

  VarExternal * stackvar;
  root->addChild(stackvar= new VarExternal((multi_stack->getSettingsFileName() + ".xml").c_str(),multi_stack->getName()));
  stackvar->addChild(multi_stack->getSettings());
  //create tabs, GL visualizations and tool-panes for each capture thread in the multi-stack:
  for (unsigned int i=0;i<multi_stack->threads.size();i++) {
    VisionStack * s = multi_stack->threads[i]->getStack();
    if (affinity!=0) multi_stack->threads[i]->setAffinityManager(affinity);

    GLWidget * gl=new GLWidget(0,false);
    gl->setRingBuffer(multi_stack->threads[i]->getFrameBuffer());
    gl->setVisionStack(s);
    QString label = "Thread " + QString::number(i);
#ifdef CAMERA_SPLITTER
    if(i == multi_stack->threads.size() - 1)
    {
      label = "Distributor Thread";
    }
#endif

    VarList * threadvar = new VarList(label.toStdString());

    threadvar->addChild(s->getSettings());
    //iterate through plugin variables

    QSplitter * stack_widget = new QSplitter(Qt::Horizontal);
    stack_widget->setOpaqueResize(true);
    stack_widget->setChildrenCollapsible(false);
    stack_widget->setHandleWidth(8);
    stack_widgets.push_back(stack_widget);
    QSplitter * stack_vis_splitter = new QSplitter(Qt::Vertical);
    stack_vis_splitter->setOpaqueResize(true);
    stack_vis_splitter->setChildrenCollapsible(false);
    stack_vis_splitter->setHandleWidth(6);
    stack_widget->addWidget(stack_vis_splitter);
    QTabWidget * stack_control_tab = new QTabWidget();
    stack_control_tab->setTabPosition(QTabWidget::East);
    stack_control_tab->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    stack_control_tab->setMinimumWidth(260);
    stack_widget->addWidget(stack_control_tab);
    stack_widget->setStretchFactor(0, 3);
    stack_widget->setStretchFactor(1, 1);

    VideoWidget * w=new VideoWidget(label,gl);
    display_widgets.push_back(gl);
    threadvar->addChild(multi_stack->threads[i]->getSettings());
    w->setRightPane(stack_widget, stack_control_tab);
    stack_right_panes.push_back(stack_control_tab);
    stack_sizes_before_right_hide.push_back(QList<int>());

    stack_vis_splitter->addWidget(w);
    //iterate through all plugins
    unsigned int n=s->stack.size();
    for (unsigned int j=0;j<n;j++) {
      VisionPlugin * p=s->stack[j];
      PluginVisualize * vis_plugin = dynamic_cast<PluginVisualize*>(p);
      if (vis_plugin != nullptr) {
        w->setCoordinateGraphVar(vis_plugin->getCoordinateGraphVar());
      }
      if (p->isSharedAmongStacks()) {
        if (i==0) {
          //this is a shared global plugin...
          //add it to global pane
          if (p->getSettings()!=0) stackvar->addChild(p->getSettings());

          QWidget * tmp_control = p->getControlWidget();
          if (tmp_control!=0) {
            if (right_tab==0) {
              right_tab=new QTabWidget();
              right_tab->setTabPosition(QTabWidget::East);
            }
            right_tab->addTab(tmp_control,QString::fromStdString(p->getName()));
          }
        }

      } else {
        if (p->getSettings()!=0) threadvar->addChild(p->getSettings());
        //this is a local plugin relating only to a single thread
        //add it to the context.
        QWidget * tmp_control = p->getControlWidget();
        if (tmp_control!=0) stack_control_tab->addTab(tmp_control,QString::fromStdString(p->getName()));

        QWidget * tmp_vis = p->getVisualizationWidget();
        if (tmp_vis!=0) stack_vis_splitter->addWidget(tmp_vis);
      }
    }
    stackvar->addChild(threadvar);

    cam_tabs->addTab(stack_widget, label);
  }

  if (affinity!=0) affinity->demandCore(multi_stack->threads.size());

  // Set position and size of main window:
  QSettings window_settings("RoboCup", "ssl-vision");
  window_settings.beginGroup("MainWindow");
  QPoint pos = window_settings.value("pos", QPoint(100, 100)).toPoint();
  QSize size = window_settings.value("size", QSize(800, 600)).toSize();
  window_settings.endGroup();
  resize(size);
  move(pos);

  //FINISHED STRUCTURAL TREE
  //NOW LOAD DATA:

  world.push_back(root);
  world=VarXML::read( world,"settings.xml");

  //update network output settings from xml file
  ((MultiStackRoboCupSSL*)multi_stack)->RefreshNetworkOutput();
  ((MultiStackRoboCupSSL*)multi_stack)->RefreshLegacyNetworkOutput();
  multi_stack->start();

  if (start_capture==true) {
    for (unsigned int i = 0; i < multi_stack->threads.size(); i++) {
      CaptureThread * ct = multi_stack->threads[i];
      ct->init();
    }
  }

  tree_view->setModel(tmodel);
  tmodel->setRootItems(world);
  tree_view->expandAndFocus(stackvar);
  tree_view->fitColumns();

  left_tab=new QTabWidget();
  left_tab->setTabPosition(QTabWidget::West);

  splitter->addWidget(left_tab);
  splitter->addWidget(cam_tabs);
  if (right_tab!=0) splitter->addWidget(right_tab);

  left_tab->addTab(tree_view,"Data-Tree");

  setCentralWidget(splitter); //was splitter

  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  if (right_tab != 0) splitter->setStretchFactor(2, 0);

  pane_toolbar = addToolBar("Panels");
  pane_toolbar->setMovable(false);
  pane_toolbar->setFloatable(false);
  pane_toolbar->setIconSize(QSize(16, 16));

  action_toggle_left_pane = new QAction("Toggle Left Pane", this);
  action_toggle_left_pane->setToolTip("Collapse/uncollapse the left data tree pane");
  connect(action_toggle_left_pane, SIGNAL(triggered()), this, SLOT(toggleLeftPane()));
  pane_toolbar->addAction(action_toggle_left_pane);

  action_toggle_right_pane = new QAction("Toggle Right Pane", this);
  action_toggle_right_pane->setToolTip("Collapse/uncollapse the thread controls pane (DVR/Mask/etc)");
  action_toggle_right_pane->setEnabled(!stack_right_panes.empty());
  connect(action_toggle_right_pane, SIGNAL(triggered()), this, SLOT(toggleRightPane()));
  pane_toolbar->addAction(action_toggle_right_pane);

  // Adjust timer to ~30ms (33fps) for smoother display on macOS
  // Original 10ms could cause excessive redraws and sync issues
  startTimer(30);

  // connection must be queued as the data tree is locked
  // by a mutex when the signal is triggered
  connect(save_settings_trigger, SIGNAL(signalTriggered()),
          this, SLOT(slotSaveSettings()), Qt::QueuedConnection);
}

void MainWindow::toggleLeftPane() {
  if (left_tab == nullptr || splitter == nullptr) return;
  const bool was_visible = left_tab->isVisible();
  if (was_visible) splitter_sizes_before_left_hide = splitter->sizes();
  left_tab->setVisible(!was_visible);

  if (!was_visible) {
    // Unhiding: restore sizes if we have them.
    if (!splitter_sizes_before_left_hide.isEmpty() &&
        splitter_sizes_before_left_hide.size() == splitter->count()) {
      splitter->setSizes(splitter_sizes_before_left_hide);
    }
    return;
  }

  // Hiding: give space to the remaining panes.
  QList<int> sizes = splitter->sizes();
  if (sizes.size() >= 2) {
    sizes[0] = 0;
    splitter->setSizes(sizes);
  }
}

void MainWindow::toggleRightPane() {
  if (cam_tabs == nullptr) return;
  const int idx = cam_tabs->currentIndex();
  if (idx < 0) return;
  if (idx >= (int)stack_widgets.size()) return;
  if (idx >= (int)stack_right_panes.size()) return;
  if (stack_widgets[idx] == nullptr || stack_right_panes[idx] == nullptr) return;

  QSplitter * stack_widget = stack_widgets[idx];
  QWidget * right_pane = stack_right_panes[idx];

  const bool was_visible = right_pane->isVisible();
  if (was_visible) stack_sizes_before_right_hide[idx] = stack_widget->sizes();

  right_pane->setVisible(!was_visible);
  if (!was_visible) {
    // Unhiding: restore sizes if we have them.
    const QList<int> restore_sizes = stack_sizes_before_right_hide[idx];
    if (!restore_sizes.isEmpty() && restore_sizes.size() == stack_widget->count()) {
      stack_widget->setSizes(restore_sizes);
    } else {
      // Fallback: make sure the right pane is visible with a sensible width.
      QList<int> sizes = stack_widget->sizes();
      if (sizes.size() >= 2) {
        sizes[1] = qMax(right_pane->minimumWidth(), sizes[1] > 0 ? sizes[1] : right_pane->minimumWidth());
        stack_widget->setSizes(sizes);
      }
    }
    return;
  }

  // Hiding: give space to the video/visualization side.
  QList<int> sizes = stack_widget->sizes();
  if (sizes.size() >= 2) {
    sizes[1] = 0;
    stack_widget->setSizes(sizes);
  }
}

void MainWindow::timerEvent( QTimerEvent * e) {
  (void)e;
  unsigned int n = display_widgets.size();
  RealTimeDisplayWidget * w;
  bool frame_changed;
  FrameBuffer * fb;

  for (unsigned int i=0;i<n;i++) {
    w = display_widgets[i];
    frame_changed=false;
    fb=w->getRingBuffer();
    if (fb!=0) {
      fb->lockRead();
      int cur=fb->curRead();
      if (fb->nextRead(true) != cur) frame_changed=true;
      fb->unlockRead();
    }
    w->displayLoopEvent(frame_changed,opts);
  }
}

void MainWindow::slotSaveSettings()
{
    VarXML::write(world,"settings.xml");
    QSettings window_settings("RoboCup", "ssl-vision");
    window_settings.sync();
}

void MainWindow::init() {

}

void MainWindow::closeEvent(QCloseEvent * event ) {
  (void)event;
  //Save window configuration before deleting:
  {
    QSettings window_settings("RoboCup", "ssl-vision");
    window_settings.beginGroup("MainWindow");
    window_settings.setValue("pos", pos());
    window_settings.setValue("size", size());
    window_settings.endGroup();
    window_settings.sync();
  }
}

MainWindow::~MainWindow() {
  if (affinity!=0) delete affinity;
  //FIXME: right now we don't clean up anything
  VarXML::write(world,"settings.xml");

  // Force sync of QSettings to ensure data is written to disk
  QSettings window_settings("RoboCup", "ssl-vision");
  window_settings.sync();

  // Stop stack:
  multi_stack->stop();
  delete multi_stack;
  exit(0);
}
