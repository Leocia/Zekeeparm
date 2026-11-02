# ADR-001: Use zero distortion for the active Gemini2 YUYV stream

## Status

Accepted

## Date

2026-07-28

## Context

The Gemini2 1280×720@30 YUYV stream is geometrically close to rectified, but
the driver previously passed the device's factory distortion coefficients
directly to OpenCV pose estimation. On a live ChArUco observation, those
coefficients produced 3.92 px reprojection RMSE and 12.22 px maximum error.
Using the same camera matrix with zero distortion reduced the errors to
0.324 px RMSE and 0.814 px maximum.

The incorrect model distorted target depth and orientation enough that a
horizontal table transformed into `base_link` with about 24–25 degrees of
tilt after hand-eye calibration.

## Decision

The project configuration selects `camera.color_distortion_mode: zero` for
the active Gemini2 YUYV stream. The driver retains an explicit `factory`
option for a future stream that is verified to be raw and unrectified.

Hand-eye collection additionally rejects ChArUco observations above the
configured reprojection RMSE threshold and rejects solved calibrations whose
fixed-target closure residual exceeds the configured translation or rotation
limit.

## Alternatives Considered

### Always use factory coefficients

Rejected because the measured reprojection error is an order of magnitude
worse on the active stream.

### Hard-code zero distortion for every Orbbec profile

Rejected because another resolution, format, or firmware path may expose an
unrectified image. An explicit mode preserves that option.

### Compensate the resulting hand-eye transform manually

Rejected because a global offset cannot correct pose-dependent distortion and
would hide invalid calibration data.

## Consequences

- The active intrinsics file must store the same runtime distortion used
  during calibration and grasping.
- A change of camera profile, format, firmware, or alignment mode requires
  repeating the reprojection comparison before retaining zero distortion.
- Hand-eye result files contain raw samples and closure residuals so future
  quality checks can be performed offline.
