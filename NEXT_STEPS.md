# What to do next (updated 2026-11-20)

**The code:** `capabilities/place_container` in rammp-org/rammp-kitchen-capability-engines.
Branch `feature/7-place-container-reach` (fork), commit `4a73f96`, **draft PR #71** into `dev`. Merged before: #33
(`feature/7-place-container`). **Robot: gen_1_5**, from snorlax, worktree `~/rammp-wt/walter-place`,
image `place_container:walter`, data `~/.place_container_gen15`.
**Where it stands:** places reliably across most of the table (10 of 11 on Nov 20); the UI works up
to the screen. Details: `PROGRESS_2026-11-20.md`. Commands for a session: `TESTING_TOMORROW.md` in the
kitchen repo; state for whoever picks it up: `HANDOFF_place_container.md` there.

## Next session
1. [ ] **Make two settings defaults** (they were only `-p` on Nov 20, so the UI doesn't get them):
       `max_table_tilt_deg` 9.0 (the arm's tilt makes a level table look 7° tilted) and `reach`
       `[0.5, 1.1]` (the arm now places from 0.5 m). Code + `rammp-alternative.yaml` + docs.
2. [ ] **Robustness runs**, a bag each, note result + cause of every failure:
   - the untested spots (0.53, 0.50), (0.62, -0.10), (0.53, -0.10);
   - repeats across the table (≥ 3 per area), different start poses;
   - **chained starts**: right after "Close fridge" and after "Grab tupperware from microwave"
     (dry-run from those end poses first; find which check refuses and why);
   - re-run (0.80, 0.35) to confirm the after-sag force fix on the arm (Nov 20 03:05 false contact).
3. [ ] **UI end to end** once the bridge is fixed: tap table → spot → Place here, and Use marked spot.
4. [ ] **Mark PR #71 ready for review** into `dev` (template sections; results with evidence; what wasn't tested).
       Before: `make check`, `make lint`, `make test-arm` (the model tests skip in the image).
5. [ ] Fix the 4 failing tests in `test/test_plan.py` ("lower: the arm can only go down…";
       Swapnil asked on PR #33) — I want to do these myself.

## Waiting on others
- [ ] **UI (Alex / David):** ui_bridge can't list the app's Remote Control presets (404 on
      `RCP_RammsBridge`), so no objects or panels reach the screen. Asked in the student chat.
- [ ] **TF (robot owner):** odom3d has no `odom` frame → `/perception/object_detections` empty.
- [ ] **Mount (robot owner / Swapnil):** gen_1_5's `mount.yaml` looks ~4–6° off in roll and pitch
      (plane fit + the camera agree on 7°). After a fix: re-mark, re-save the start.

## Later
- Dockerfile: install the deps before copying the code, so a code change rebuilds in seconds.
- Offer only reachable spots in the UI picker (a quick IK check per spot before drawing its box).
- cuRobo return around the bowl and table, if the sideways back-out ever isn't enough.
- Remove `allow_reconfigure` (old planner only; the level place never reads it).

## History
`PROGRESS_2026-09-29.md` … `PROGRESS_2026-10-29.md` (flatbot, the port), `PROGRESS_2026-11-10.md`
(fast, smooth place on flatbot), **`PROGRESS_2026-11-20.md`** (gen_1_5: reach, force contact, the arm
tilt, UI blockers). Design: `PLAN_2026-10-06_scene_camera.md`.
