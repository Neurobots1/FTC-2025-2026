# Auto Commands Cheat Sheet

This is the quick reference for the modular autonomous builder:

`ModularAutoBuilder builder = new ModularAutoBuilder(robot);`

## Core Commands

`doAction(action)`
- Add any raw `AutoAction`.

`follow(path, speed, holdEnd)`
- Start following a Pedro path and wait until the follower is no longer busy.

`followAsync(path, speed, holdEnd)`
- Start following a Pedro path immediately and continue to the next command without waiting.

`waitForFollowerIdle()`
- Wait until Pedro is no longer busy.

`waitSeconds(seconds)`
- Wait for a fixed amount of time.

`waitForSortedShotZone()`
- Wait until the robot is inside the sorted shooter's valid shooting zone.

`waitForSortedReadyToShoot()`
- Wait until the sorted shooter is in zone and the flywheel/gate are ready.

`waitForUnsortedShotZone()`
- Wait until the robot is inside the unsorted shooter’s valid shooting zone.

`waitForUnsortedReadyToShoot()`
- Wait until the unsorted shooter is in zone and at firing speed.

## Sorted Auto Commands

`readSortPattern(aprilTag, timeoutSeconds, fallbackPattern)`
- Read the AprilTag and lock the sorted pattern (`PGP`, `GPP`, `PPG`, `NOSORT`).

`lockSortPattern(aprilTag, timeoutSeconds, fallbackPattern)`
- Same as `readSortPattern(...)`.

`shootSortedPreload(toShotPose)`
- Pre-spin, drive to the preload shot pose, shoot preload, wait until the sorted controller is idle.

`shootPreloadSorted(toShotPose)`
- Same as `shootSortedPreload(...)`.

`collectSortedLine(line, toLineStart, toLineFinish)`
- Start sorted intake for a line, drive to the line, and wait until the sorted controller is idle.

`collectSortedLine(line, toLineStart, toLineFinish, finishSpeed)`
- Same as above, with a custom finish speed on the last path.

`shootSortedLine(line, toShotPose)`
- Drive to the shot pose, shoot that sorted line, wait until the sorted controller is idle.

`sortedCycle(line, toLineStart, toLineFinish, toShotPose)`
- Shortcut for collect + shoot with default finish speed `1.0`.

`sortedCycle(line, toLineStart, toLineFinish, finishSpeed, toShotPose)`
- Shortcut for collect + shoot with custom finish speed.

`stopSortedShooter()`
- Turn off sorted pre-spin.

`startSortedIntake(line)`
- Start sorted intake for a logical line immediately.

`startSortedShot(line)`
- Start sorted shooting for a logical line immediately.

`waitForSortedIdle()`
- Wait until the sorted controller finishes its current sequence.

## Unsorted Auto Commands

`shootUnsorted(toShotPose)`
- Enable unsorted shooter, drive to the shot pose, auto-feed, and wait until shooting is done.

`enableUnsortedShooter(true / false)`
- Turn the unsorted shooter on or off from the builder.

`startUnsortedShot()`
- Start an unsorted shot immediately.

`waitForUnsortedShotDone()`
- Wait until the unsorted shot sequence finishes.

`stopUnsortedShooter()`
- Turn off the unsorted shooter.

## Useful Low-Level Actions Through `robot`

These are usually used with `doAction(...)`.

`robot.enableSortedPreSpin(true / false)`
- Turn sorted pre-spin on or off.

`robot.startSortedIntake(line)`
- Start sorted intake on a logical line.

`robot.startSortedShot(line)`
- Start sorted shooting for a logical line.

`robot.startSortedPreloadShot()`
- Start preload shooting in sorted mode.

`robot.waitForSortedIdle()`
- Wait until the sorted controller finishes.

`robot.enableUnsortedShooter(true / false)`
- Turn the unsorted shooter on or off.

`robot.startUnsortedShot()`
- Start auto-feeding in unsorted mode.

`robot.waitForUnsortedShotDone()`
- Wait until the unsorted shot is finished.

## Common Patterns

### Unsorted, simple

```java
builder
    .shootUnsorted(toPreloadShot)
    .follow(toLine2Start, 1.0, false)
    .follow(toLine2Finish, 1.0, false)
    .doAction(startTimedIntake())
    .waitSeconds(0.9)
    .doAction(stopIntake())
    .shootUnsorted(line2ToShot)
    .stopUnsortedShooter();
```

### Explicit wait for Pedro

```java
builder
    .followAsync(toGate, 1.0, true)
    .waitForFollowerIdle()
    .doAction(startTimedIntake())
    .waitSeconds(1.0)
    .doAction(stopIntake());
```

### Fire when in shot zone instead of waiting for path end

```java
builder
    .enableUnsortedShooter(true)
    .followAsync(toMovingShot, 1.0, true)
    .waitForUnsortedReadyToShoot()
    .startUnsortedShot()
    .waitForUnsortedShotDone();
```

### Sorted, simple

```java
builder
    .readSortPattern(aprilTag, 1.0, SortPattern.NOSORT)
    .shootSortedPreload(toPreloadShot)
    .sortedCycle(1, toLine1Start, toLine1Finish, 1.0, line1ToShot)
    .sortedCycle(2, toLine2Start, toLine2Finish, 1.0, line2ToFinalShot)
    .stopSortedShooter();
```

### Sorted, fully explicit

```java
builder
    .readSortPattern(aprilTag, 1.0, SortPattern.NOSORT)
    .followAsync(toLine1Start, 1.0, true)
    .waitForFollowerIdle()
    .followAsync(toLine1Finish, 1.0, true)
    .waitForFollowerIdle()
    .doAction(robot.enableSortedPreSpin(true))
    .doAction(robot.startSortedIntake(1))
    .doAction(robot.waitForSortedIdle())
    .followAsync(line1ToShot, 1.0, true)
    .waitForFollowerIdle()
    .doAction(robot.startSortedShot(1))
    .doAction(robot.waitForSortedIdle())
    .stopSortedShooter();
```

## Main Rule

Use `follow(...)` when you want:
- "Drive there, then continue."

Use `followAsync(...).waitForFollowerIdle()` when you want:
- the wait to be written explicitly
- overlap between starting motion and other commands
- more control over exactly when the script pauses

Use `waitForUnsortedShotZone()` or `waitForUnsortedReadyToShoot()` when you want:
- to fire as soon as the shooter is geometrically valid
- to shoot on the move without waiting for Pedro to finish the full path










```

```




