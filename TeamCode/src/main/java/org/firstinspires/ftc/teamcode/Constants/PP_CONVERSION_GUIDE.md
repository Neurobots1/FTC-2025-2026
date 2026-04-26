# PP Conversion Guide

This guide explains how to:

1. turn autonomous Java path code into a visualizer `.pp` file
2. turn a visualizer `.pp` file back into Java path code

The examples below match this repo's current autonomous structure, especially:

- [Unsorted15BallAuto.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpMode/Autonomous/Unsorted/balls15/Unsorted15BallAuto.java)
- [AutoPoseConstants.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants/AutoPoseConstants.java)
- [MirroredPathFactory.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Autonomous/Modular/MirroredPathFactory.java)
- [AllianceMirroring.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Autonomous/Modular/AllianceMirroring.java)

## Big Picture

In this repo:

- Java is the source of truth for robot behavior.
- `.pp` files are a visualizer-friendly representation of path geometry, headings, and waits.
- Alliance mirroring happens in Java, not by manually writing separate red constants.
- Turret-enabled vs turret-disabled changes the heading behavior and sometimes the path type.

## File Locations

Useful examples:

- [Unsorted15BallAutoBlue.pp](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/doc/visualizer/Unsorted15BallAutoBlue.pp)
- [Unsorted15BallAutoRed.pp](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/doc/visualizer/Unsorted15BallAutoRed.pp)
- [Unsorted15BallAutoBlueTurretEnabled.pp](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/doc/visualizer/Unsorted15BallAutoBlueTurretEnabled.pp)
- [Unsorted15BallAutoBlueTurretDisabled.pp](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/doc/visualizer/Unsorted15BallAutoBlueTurretDisabled.pp)
- [Unsorted15BallAutoRedTurretEnabled.pp](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/doc/visualizer/Unsorted15BallAutoRedTurretEnabled.pp)
- [Unsorted15BallAutoRedTurretDisabled.pp](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/doc/visualizer/Unsorted15BallAutoRedTurretDisabled.pp)

## Java To `.pp`

### Step 1: Find the blue-side pose constants

Start with the auto class and its constants.

Example from [Unsorted15BallAuto.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpMode/Autonomous/Unsorted/balls15/Unsorted15BallAuto.java):

- `startPose = AutoPoseConstants.CloseStartPose()`
- `preloadShot = AutoPoseConstants.CloseShootPose()`
- `line2Start = AutoPoseConstants.line2StartPose()`
- `line2Finish = AutoPoseConstants.line2FinishPoseUnsorted()`
- `line2Control = AutoPoseConstants.line2ControlPose()`
- `gatePose = AutoPoseConstants.gatePose()`
- `gateControl = AutoPoseConstants.gateControlPose()`
- `line1Start = AutoPoseConstants.line1StartPose()`
- `line1Finish = AutoPoseConstants.line1FinishPose()`
- `finalShot = AutoPoseConstants.finalShotPose()`
- `gateShootControl = AutoPoseConstants.gateShootControl()`

Then read the numeric values from [AutoPoseConstants.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants/AutoPoseConstants.java).

### Step 2: Determine the path builder used

Look at which helper built each path in `buildPaths()`.

These map directly:

- `paths().line(...)` -> `.pp` line with `heading: "linear"`
- `paths().curve(...)` -> `.pp` curve with control point(s) and `heading: "linear"`
- `paths().tangentLine(...)` -> `.pp` line with `heading: "tangential"`
- `paths().tangentCurve(...)` -> `.pp` curve with control point(s) and `heading: "tangential"`
- `paths().shotLine(...)` -> `.pp` line with `heading: "linear"` unless turret-disabled lock logic changes your interpretation
- `paths().shotCurve(...)` -> `.pp` curve with `heading: "linear"` unless turret-enabled goal-facing is being approximated

Important detail from [MirroredPathFactory.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Autonomous/Modular/MirroredPathFactory.java):

- `tangentLine` and `tangentCurve` use `setTangentHeadingInterpolation()`
- normal `line` and `curve` use linear heading interpolation from start pose heading to end pose heading
- `shotLine` and `shotCurve` may face the goal when `useGoalFacing` is enabled

### Step 3: Convert heading behavior

Use this cheat sheet:

#### Linear heading

If Java uses:

```java
paths().line(start, end)
paths().curve(start, control, end)
```

then `.pp` end point should usually look like:

```json
"endPoint": {
  "x": 50,
  "y": 55,
  "heading": "linear",
  "startDeg": 139,
  "endDeg": 180
}
```

Use the headings from the Java start and end poses.

#### Tangential heading

If Java uses:

```java
paths().tangentLine(start, end)
paths().tangentCurve(start, control, end)
```

then `.pp` end point should usually look like:

```json
"endPoint": {
  "x": 50,
  "y": 55,
  "heading": "tangential",
  "reverse": false
}
```

#### Shot paths

If Java uses:

```java
paths().shotLine(...)
paths().shotCurve(...)
```

then check turret mode:

- turret disabled: usually encode this like a normal linear-heading shot path
- turret enabled: the real Java path may be goal-facing during motion, which the visualizer cannot represent perfectly with one simple heading number

Practical rule for this repo:

- keep the geometry correct
- for turret-enabled `.pp`, use tangential where the path is meant to be "drive naturally"
- keep shot return/final shot headings readable and consistent for visualization

That is why the explicit turret-enabled `.pp` files are approximations of runtime goal-facing behavior, not a perfect simulation of Pedro heading interpolation.

### Step 4: Convert geometry

Map the path shape:

- Java `BezierLine(start, end)` -> `.pp` with no control points
- Java `BezierCurve(start, control, end)` -> `.pp` with one control point

Example:

```java
toLine2Start = paths().curve(preloadShot, line2Control, line2Start);
```

becomes:

```json
{
  "id": "line-preload-to-line2-start",
  "name": "Preload Shot to Line 2 Start",
  "endPoint": {
    "x": 50,
    "y": 55,
    "heading": "linear",
    "startDeg": 139,
    "endDeg": 180
  },
  "controlPoints": [
    { "x": 55, "y": 50 }
  ]
}
```

### Step 5: Build the sequence from `buildRoutine()`

The visualizer `sequence` should follow the `followAsync(...)` order in the routine.

Example from [Unsorted15BallAuto.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpMode/Autonomous/Unsorted/balls15/Unsorted15BallAuto.java):

- `followAsync(toPreloadShot...)`
- `followAsync(toLine2Start...)`
- `followAsync(toLine2Finish...)`
- `followAsync(line2ToShot...)`
- `followAsync(shotToGate...)`
- `followAsync(gateToShot...)`
- `followAsync(shotToGateAgain...)`
- `followAsync(gateToFinalShot...)`
- `followAsync(shotToLine1Start...)`
- `followAsync(line1StartToFinish...)`
- `followAsync(line1ToFinalShot...)`

Add waits for actions like:

- `Actions.waitSeconds(GATE_INTAKE_SECONDS)` -> `1500 ms`
- intake settle pauses you want to show in the visualizer

### Step 6: Mirror for red if needed

Java mirrors red automatically using [AllianceMirroring.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/Autonomous/Modular/AllianceMirroring.java).

For `.pp`, you must store actual mirrored coordinates.

Practical method:

1. make the blue `.pp` first
2. duplicate and mirror it in the visualizer
3. rename it as the red version

That is safer than hand-editing every coordinate.

## `.pp` To Java

### Step 1: Read start point and each line

For each `.pp` line, capture:

- end point
- control point(s)
- heading mode
- sequence order

Then create matching Java `Pose` constants or reuse existing ones in [AutoPoseConstants.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants/AutoPoseConstants.java).

### Step 2: Pick the correct Java builder

Use this reverse mapping:

- no control points + `heading: "linear"` -> `paths().line(...)` or `paths().shotLine(...)`
- has control point(s) + `heading: "linear"` -> `paths().curve(...)` or `paths().shotCurve(...)`
- no control points + `heading: "tangential"` -> `paths().tangentLine(...)`
- has control point(s) + `heading: "tangential"` -> `paths().tangentCurve(...)`

If the path is meant to be a shooting path, use `shotLine` or `shotCurve`.

### Step 3: Recover pose headings

When `.pp` uses:

- `heading: "linear"` with `startDeg` and `endDeg`

then create Java poses with those headings in radians:

```java
new Pose(x, y, Math.toRadians(deg))
```

When `.pp` uses:

- `heading: "tangential"`

then Java usually does not need explicit heading interpolation values on that segment because `tangentLine` or `tangentCurve` handles it.

Still, the endpoint `Pose` heading in your constants should usually remain meaningful for the next segment and for shot paths.

### Step 4: Rebuild `buildPaths()`

Example pattern:

```java
toPreloadShot = paths().shotLine(startPose, preloadShot);
toLine2Start = paths().curve(preloadShot, line2Control, line2Start);
toLine2Finish = paths().line(line2Start, line2Finish);
```

For turret branches, split it the same way as [Unsorted15BallAuto.java](/C:/Users/Neurobots/StudioProjects/FTC-2025-2026/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpMode/Autonomous/Unsorted/balls15/Unsorted15BallAuto.java):

- `buildTurretEnabledPaths(...)`
- `buildTurretDisabledPaths(...)`

### Step 5: Rebuild `buildRoutine()`

The `.pp` `sequence` maps to `followAsync(...)` calls in order.

The `.pp` waits map to either:

- `Actions.waitSeconds(...)`
- or an action boundary where the robot is intentionally pausing

Keep mechanism actions in Java even though the `.pp` only shows the timing.

Examples:

- intake on/off
- shooter enable/disable
- shoot one ball
- wait for ready to shoot

## Recommended Workflow

### Code to `.pp`

1. Read `buildPaths()`.
2. Read the used constants.
3. Make the blue `.pp`.
4. Add waits from `buildRoutine()`.
5. Duplicate and mirror for red.
6. If turret logic branches, export one `.pp` per branch.

### `.pp` to code

1. Read the `.pp` sequence and geometry.
2. Define or update pose constants.
3. Recreate `buildPaths()` with the matching `paths()` helper.
4. Recreate `buildRoutine()` from sequence order and waits.
5. Put alliance mirroring in Java instead of hardcoding both blue and red constants unless the auto truly differs by alliance.

## Repo-Specific Rules

- Blue constants are the base constants.
- Red is usually generated by `Pose.mirror()` through the alliance helper.
- Turret-enabled and turret-disabled can require separate path definitions.
- The visualizer `.pp` format is best treated as a geometry and timing artifact, not a perfect dump of every runtime Java behavior.

## Common Mistakes

- Forgetting that red is mirrored in Java.
- Encoding a tangent path as linear in `.pp`.
- Forgetting to add control points for curves.
- Treating `shotCurve` goal-facing behavior as if it were the same as a plain linear heading.
- Converting waits but forgetting the corresponding mechanism actions in Java.
- Changing `.pp` geometry without updating the pose constants used by other autos.

## Fast Checklist

When converting Java to `.pp`:

- Did I copy the right coordinates from `AutoPoseConstants`?
- Did I choose `linear` vs `tangential` correctly?
- Did I include control points for curves?
- Did I preserve follow order from `buildRoutine()`?
- Did I add the waits?
- Did I export separate files for turret branches if needed?

When converting `.pp` to Java:

- Did I choose the correct `paths()` helper?
- Did I reconstruct headings in radians?
- Did I rebuild the routine order?
- Did I keep alliance mirroring in Java?
- Did I separate turret-enabled and turret-disabled path builds where necessary?

