# Vision pre-match checklist

Run through this in the pits before every match.

- [ ] **Field layout matches the event** on both the Orange Pi (PhotonVision settings) and the
      robot code (`VisionConstants.kFieldLayout`): Welded or AndyMark. A mismatch is the most
      common vision failure.
- [ ] **Both camera streams are live** in the PhotonVision web UI (`http://10.5.37.11:5800`), and
      the camera names are `front-left` and `front-right`.
- [ ] **Exposure looks right for this venue.** Tags should be crisp black and white. If an OV9281
      stream looks wrong, toggle exposure once (known camera quirk).
- [ ] **Dashboard `Vision/<camera>/connected` is true** for both cameras.
- [ ] **`Vision/<camera>/rejected` is not climbing** while the robot sits still with a tag in view.
- [ ] **The robot's pose on the dashboard field matches where the robot actually is.**
