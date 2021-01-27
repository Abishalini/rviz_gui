#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "visualization_manager.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/display.hpp"

#include "myviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{
  // Construct and lay out labels and slider controls.
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );

  // Construct and lay out render panel.
  render_panel_ = new rviz_common::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abstraction_ = std::weak_ptr<rviz_common::ros_integration::RosNodeAbstractionIface>();
  rviz_common::WindowManagerInterface* wm_ = nullptr;
  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz_common::VisualizationManager( render_panel_ , ros_node_abstraction_, wm_,  clock_);
  //render_panel_->initialize( manager_->getSceneManager(), manager_ );
  render_panel_->initialize(manager_ );
  manager_->initialize();
  manager_->startUpdate();

  // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  assert( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( QColor( Qt::yellow ) );

  // Initialize the slider values.
  thickness_slider->setValue( 25 );
  cell_size_slider->setValue( 10 );
}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}