#ifndef RENDERER_H
#define RENDERER_H

#include "autoc/rpc/protocol.h"

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPolyLine.h>
#include <vtkObjectFactory.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkPlaneSource.h>
#include <vtkCellData.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleUser.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkArrowSource.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkTubeFilter.h>
#include <vtkRibbonFilter.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkRendererCollection.h>
#include <array>
#include <chrono>

#define FIELD_SIZE 100.0
#define FIELD_GAP 10.0

// Forward declarations
class Renderer;

// Custom event IDs
enum {
  NextModelEvent = vtkCommand::UserEvent + 1,
  PreviousModelEvent,
  NewestModelEvent,
  OldestModelEvent,
  NextTestEvent,
  PreviousTestEvent,
  AllFlightEvent,
  PlaybackEvent
};

// CustomInteractorStyle forward declaration
class CustomInteractorStyle;

// Extern declaration for xiao-only mode flag (defined in renderer.cc)
extern bool inXiaoOnlyMode;

// Forward declarations for timestamped structures (defined in renderer.cc)
struct TimestampedVec;

// Test span structure for xiao autoc-engaged segments
struct TestSpan {
  size_t startIndex;
  size_t endIndex;
  unsigned long startTime;
  unsigned long endTime;
  gp_vec3 origin;  // Test origin for xiao mode
  std::vector<TimestampedVec> vecPoints;  // Vec arrows for this span
  int pathIndex;  // Path index from GP State (0-5)

  TestSpan() : startIndex(0), endIndex(0), startTime(0), endTime(0), origin(0.0f, 0.0f, 0.0f), pathIndex(-1) {}
  TestSpan(size_t start, size_t end, unsigned long stime, unsigned long etime)
    : startIndex(start), endIndex(end), startTime(stime), endTime(etime), origin(0.0f, 0.0f, 0.0f), pathIndex(-1) {}
  TestSpan(size_t start, size_t end, unsigned long stime, unsigned long etime, gp_vec3 orig)
    : startIndex(start), endIndex(end), startTime(stime), endTime(etime), origin(orig), pathIndex(-1) {}
};

class Renderer {
public:
  void initialize();
  bool isRunning();
  bool updateGenerationDisplay(int genNumber);
  void updateTextDisplay(int generation, gp_fitness fitness);
  void jumpToNewestGeneration();
  void jumpToOldestGeneration();
  void nextTest();
  void previousTest();
  void showAllFlight();
  void togglePlaybackAnimation();
  void updatePlaybackAnimation();
  void pausePlaybackAnimation();
  void resumePlaybackAnimation();
  void renderFullScene(); // Render complete scene without S3 fetch
  void hideStopwatch();
  void updateStopwatchPosition();
  void toggleFocusMode();
  void adjustFocusArena(int delta);
  void focusMoveLeft();
  void focusMoveRight();
  void focusMoveUp();
  void focusMoveDown();

  int genNumber = 0;

  // Store current generation and fitness for resize updates
  int currentGeneration = 0;
  gp_fitness currentFitness = 0.0;

  // 030 M9a — Set true at dmp load when EvalResults v=2 fields are
  // populated (cameraViewList + targetTrajectoryList non-empty), false
  // otherwise. Drives downstream M9b/c/d render-path branching: when
  // true, instantiate target-craft + beacon + camera-POV actors;
  // when false, render the existing pathgen-only view unchanged.
  bool isTrackerMode_ = false;

  // 030 M9b detail-toggle 2026-05-08: gates the M9b.2/M9b.3 debug-aid
  // overlays (target-wingtip beacon glyphs + chase camera FOV pyramid
  // wireframe). Default off — operator presses 'd' to show during
  // troubleshooting. Tape ribbons + chase→target error bars remain
  // always-visible. M1 pathgen mode unaffected.
  bool trackerDetailVisible_ = false;

public:
  // 030 M9b — Toggle the detail-overlay actors. Called from the 'd'
  // key handler in CustomInteractorStyle.
  void toggleTrackerDetail();
  
  // Test span navigation state
  std::vector<TestSpan> testSpans;
  int currentTestIndex = 0;
  bool showingFullFlight = false;
  bool inXiaoMode = false;
  bool focusMode = false;
  int focusArenaIndex = 0;
  std::array<gp_scalar,3> focusCameraPosition{0.0f,0.0f,0.0f};
  std::array<gp_scalar,3> focusCameraFocalPoint{0.0f,0.0f,0.0f};
  std::array<gp_scalar,3> focusCameraViewUp{0.0f,0.0f,-1.0f};
  
  // Animation state
  bool isPlaybackActive = false;
  bool isPlaybackPaused = false;
  std::chrono::steady_clock::time_point animationStartTime;
  std::chrono::steady_clock::time_point pauseStartTime;
  std::chrono::duration<gp_scalar> totalPausedTime = std::chrono::duration<gp_scalar>::zero();
  gp_scalar animationSpeed = 1.0f; // seconds per animation second
  gp_scalar totalAnimationDuration = 10.0f; // total animation duration in seconds
  unsigned long animationTimerId = 0; // VTK timer ID for animation

  vtkSmartPointer<vtkRenderWindow> renderWindow;
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;

private:
  vtkSmartPointer<vtkOrientationMarkerWidget> orientationWidget;
  vtkSmartPointer<vtkAppendPolyData> paths;
  vtkSmartPointer<vtkAppendPolyData> actuals;
  vtkSmartPointer<vtkAppendPolyData> segmentGaps;
  vtkSmartPointer<vtkAppendPolyData> planeData;
  vtkSmartPointer<vtkAppendPolyData> blackboxTapes;
  vtkSmartPointer<vtkAppendPolyData> blackboxHighlightTapes;  // For highlighted test spans
  vtkSmartPointer<vtkAppendPolyData> xiaoVecArrows;  // For xiao vec vectors
  vtkSmartPointer<vtkAppendPolyData> directRabbitData;  // Direct rabbit from log rabbit=[x,y,z]

  // 030 M9b — Target-craft trajectory tape, populated from
  // targetTrajectoryList when isTrackerMode_ at load time. Mirrors
  // `actuals` (chase-craft tape); shares createTapeSet() pipeline.
  // Distinct color (orange) so chase + target read as two ribbons in
  // the 3rd-person view. Empty + invisible for pathgen-mode dmps.
  vtkSmartPointer<vtkAppendPolyData> targetActuals;

  // 030 M9b.2 — Per-tick beacon world positions on target wingtips,
  // navigation-light convention (RED port = left, GREEN starboard =
  // right). Glyphed as small spheres. Per-tick trail visualizes target
  // craft body roll — beacons trace banking turns, providing
  // orientation context beyond what the tape ribbon alone shows.
  vtkSmartPointer<vtkAppendPolyData> targetBeaconsLeft;
  vtkSmartPointer<vtkAppendPolyData> targetBeaconsRight;

  // 030 M9b.3 — Chase camera FOV pyramid frustum, drawn from the
  // latest visible tick's CameraViewSample (camera_pose_world_pos +
  // _orient + fov_h/v). Wireframe lines: 4 edges from apex to corners
  // at length 30m, plus 4 base lines closing the rectangular frustum.
  // Visualizes "where is chase looking?" — operator can eyeball
  // whether target sits inside or outside chase's FOV at any tick.
  vtkSmartPointer<vtkAppendPolyData> chaseCameraFov;

  vtkSmartPointer<vtkActor> actor1;
  vtkSmartPointer<vtkActor> directRabbitActor;  // Magenta tube for direct rabbit ground truth
  vtkSmartPointer<vtkActor> actor2;
  vtkSmartPointer<vtkActor> actor3;
  vtkSmartPointer<vtkActor> blackboxActor;
  vtkSmartPointer<vtkActor> blackboxHighlightActor;  // For highlighted test spans
  vtkSmartPointer<vtkActor> xiaoVecActor;  // For xiao vec arrows
  vtkSmartPointer<vtkActor> targetActor;   // 030 M9b — orange tape from targetTrajectoryList
  vtkSmartPointer<vtkActor> targetBeaconLeftActor;   // 030 M9b.2 — red sphere trail (port wingtip)
  vtkSmartPointer<vtkActor> targetBeaconRightActor;  // 030 M9b.2 — green sphere trail (starboard wingtip)
  vtkSmartPointer<vtkActor> chaseCameraFovActor;     // 030 M9b.3 — yellow wireframe FOV pyramid
  std::vector<vtkSmartPointer<vtkActor>> arenaLabelActors;
  
  vtkSmartPointer<vtkTextActor> generationTextActor;
  vtkSmartPointer<vtkTextActor> generationValueActor;
  vtkSmartPointer<vtkTextActor> fitnessTextActor;
  vtkSmartPointer<vtkTextActor> fitnessValueActor;
  vtkSmartPointer<vtkTextActor> testTextActor;
  vtkSmartPointer<vtkTextActor> testValueActor;
  
  // Stopwatch components
  vtkSmartPointer<vtkActor2D> stopwatchActor;
  vtkSmartPointer<vtkTextActor> stopwatchTimeActor;
  bool stopwatchVisible = false;
  gp_scalar stopwatchTime = 0.0f;

  // Control HUD components (stick + throttle)
  vtkSmartPointer<vtkActor2D> controlOutlineActor;
  vtkSmartPointer<vtkActor2D> controlStickActor;
  vtkSmartPointer<vtkActor2D> throttleFillActor;
  vtkSmartPointer<vtkTextActor> controlSourceActor;
  vtkSmartPointer<vtkActor2D> attitudeSkyActor;
  vtkSmartPointer<vtkActor2D> attitudeGroundActor;
  vtkSmartPointer<vtkActor2D> attitudeOutlineActor;
  vtkSmartPointer<vtkTextActor> velocityActor;
  bool controlsVisible = false;
  gp_scalar lastControlPitch = 0.0f;
  gp_scalar lastControlRoll = 0.0f;
  gp_scalar lastControlThrottle = 0.0f;
  gp_scalar lastControlsTime = 0.0f;

  gp_vec3 renderingOffset(int i); // locate a coordinate offset for our rendering screen
  vtkSmartPointer<vtkPolyData> createPointSet(gp_vec3 offset, const std::vector<gp_vec3> points);
  vtkSmartPointer<vtkPolyData> createPointSet(gp_vec3 offset, const std::vector<gp_vec3> points, gp_scalar timeProgress);
  vtkSmartPointer<vtkPolyData> createSegmentSet(gp_vec3 offset, const std::vector<AircraftState> state, const std::vector<gp_vec3> end);
  vtkSmartPointer<vtkPolyData> createSegmentSet(gp_vec3 offset, const std::vector<AircraftState> state, const std::vector<gp_vec3> end, gp_scalar timeProgress);

  // 030 M9b — Tracker-mode segment-set: per-tick line from chase position
  // to TARGET craft position (CopiedTargetSample.position). Pathgen's
  // createSegmentSet uses chase.getRabbitPosition() (path-following rabbit);
  // tracker mode wants chase→target ground-truth visualization since the
  // target is a real craft, not a synthetic path point. iterates in
  // lockstep to min(state.size(), targets.size()).
  vtkSmartPointer<vtkPolyData> createSegmentSetToTarget(gp_vec3 offset,
      const std::vector<AircraftState>& state,
      const std::vector<CopiedTargetSample>& targets);
  vtkSmartPointer<vtkPolyData> createTapeSet(gp_vec3 offset, const std::vector<gp_vec3> points, const std::vector<gp_vec3> normals);
  vtkSmartPointer<vtkPolyData> createTapeSet(gp_vec3 offset, const std::vector<gp_vec3> points, const std::vector<gp_vec3> normals, gp_scalar timeProgress);
  std::vector<gp_vec3> pathToVector(const std::vector<Path> path);
  std::vector<gp_vec3> stateToVector(const std::vector<AircraftState> path);
  std::vector<gp_vec3> stateToOrientation(const std::vector<AircraftState> state);

  // 030 M9b — Tracker-mode target-craft tape helpers. Symmetric with
  // stateToVector / stateToOrientation but consume CopiedTargetSample
  // (targetTrajectoryList[scenario][tick]) instead of AircraftState.
  // Returns world-frame positions / body-up vectors so createTapeSet's
  // existing pipeline renders the target tape with the same geometry
  // as the chase tape.
  std::vector<gp_vec3> targetSamplesToVector(const std::vector<CopiedTargetSample>& samples);
  std::vector<gp_vec3> targetSamplesToOrientation(const std::vector<CopiedTargetSample>& samples);

  // 030 M9b.2 — Compute per-tick beacon world positions on the target
  // craft. World pos = target.position + target.orientation × mount_body.
  // Mount offsets are hardcoded to autoc-tracker.ini v1 defaults; if
  // BeaconLeftMountY / BeaconRightMountY change in the .ini, update
  // kBeaconLeftMountBody / kBeaconRightMountBody constants in renderer.cc.
  std::vector<gp_vec3> targetSamplesToBeaconPositions(
      const std::vector<CopiedTargetSample>& samples,
      const gp_vec3& beacon_mount_body);

  // 030 M9b.3 — Build a wireframe pyramid frustum representing the
  // chase camera FOV at one tick's pose. Apex = camera_pose_world_pos
  // (+ offset). Axis = camera_pose_world_orient × +x. Half-extents at
  // distance `length` (m): tan(fov_h/2)·length on right, tan(fov_v/2)·length
  // on down. Returns vtkPolyData with 8 line segments (4 apex→corner
  // edges + 4 base-rectangle lines). Caller appends to chaseCameraFov.
  vtkSmartPointer<vtkPolyData> createFovPyramidLines(gp_vec3 offset,
      const CameraViewSample& cam,
      gp_scalar length);
  gp_fitness extractFitnessFromGP(const std::vector<char>& gpData);
  void createHighlightedFlightTapes(gp_vec3 offset);
  void createStopwatch();
  void updateStopwatch(gp_scalar currentTime);
  void createControlsOverlay();
  void updateControlsOverlay(gp_scalar currentTime);
  void updateControlsPosition();
  bool getControlStateAtTime(gp_scalar currentTime, gp_scalar& pitch, gp_scalar& roll, gp_scalar& throttle, int arenaIndex, bool& usedBlackbox, const AircraftState*& chosenState);
  void setFocusArena(int arenaIdx);
};

// CustomInteractorStyle implementation
class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static CustomInteractorStyle* New();
  vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);
  
  void SetRenderer(Renderer* renderer) { renderer_ = renderer; }

protected:
  void OnChar() override {
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    if (key == "n") {
      if (!inXiaoOnlyMode) {  // Skip generation nav in xiao-only mode
        this->InvokeEvent(NextModelEvent, nullptr);
      }
    }
    else if (key == "N") {
      if (!inXiaoOnlyMode) {
        this->InvokeEvent(NewestModelEvent, nullptr);
      }
    }
    else if (key == "p") {
      if (!inXiaoOnlyMode) {
        this->InvokeEvent(PreviousModelEvent, nullptr);
      }
    }
    else if (key == "P") {
      if (!inXiaoOnlyMode) {
        this->InvokeEvent(OldestModelEvent, nullptr);
      }
    }
    else if (key == "t") {
      this->InvokeEvent(NextTestEvent, nullptr);
    }
    else if (key == "r") {
      this->InvokeEvent(PreviousTestEvent, nullptr);
    }
    else if (key == "a") {
      this->InvokeEvent(AllFlightEvent, nullptr);
    }
    else if (key == "space") {
      this->InvokeEvent(PlaybackEvent, nullptr);
    }
    else if (key == "f") {
      if (renderer_) {
        renderer_->toggleFocusMode();
      }
    }
    else if (key == "d") {
      // 030 M9b — Toggle tracker-mode detail overlays (FOV pyramid +
      // wingtip beacon glyphs). No-op in pathgen mode (those actors
      // never have data anyway).
      if (renderer_) {
        renderer_->toggleTrackerDetail();
      }
    }
    else if (key == "Left") {
      if (renderer_) {
        renderer_->focusMoveLeft();
      }
    }
    else if (key == "Right") {
      if (renderer_) {
        renderer_->focusMoveRight();
      }
    }
    else if (key == "Up") {
      if (renderer_) {
        renderer_->focusMoveUp();
      }
    }
    else if (key == "Down") {
      if (renderer_) {
        renderer_->focusMoveDown();
      }
    }
    else {
      vtkInteractorStyleTrackballCamera::OnChar();
    }
  }
  
  void OnLeftButtonDown() override {
    if (renderer_ && renderer_->isPlaybackActive) {
      renderer_->pausePlaybackAnimation();
    }
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }
  
  void OnLeftButtonUp() override {
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
    if (renderer_ && renderer_->isPlaybackActive) {
      renderer_->resumePlaybackAnimation();
    }
  }
  
  void OnMiddleButtonDown() override {
    if (renderer_ && renderer_->isPlaybackActive) {
      renderer_->pausePlaybackAnimation();
    }
    vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
  }
  
  void OnMiddleButtonUp() override {
    vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
    if (renderer_ && renderer_->isPlaybackActive) {
      renderer_->resumePlaybackAnimation();
    }
  }
  
  void OnRightButtonDown() override {
    if (renderer_ && renderer_->isPlaybackActive) {
      renderer_->pausePlaybackAnimation();
    }
    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
  }
  
  void OnRightButtonUp() override {
    vtkInteractorStyleTrackballCamera::OnRightButtonUp();
    if (renderer_ && renderer_->isPlaybackActive) {
      renderer_->resumePlaybackAnimation();
    }
  }

private:
  Renderer* renderer_ = nullptr;
};

vtkStandardNewMacro(CustomInteractorStyle);

#endif
