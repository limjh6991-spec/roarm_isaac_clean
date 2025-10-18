# RoArm-M3 Link STL & URDF Patch Report (v1)

**Date:** 2025-10-18  
**Inputs:**
- FreeCAD: `/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/meshes/visual/roarm_m3.FCStd`
- URDF Original: `roarm_m3_v2_complete.urdf`
- URDF Patched: `roarm_m3_v2_complete.patched.urdf`

**Rules:**
- STL scale: **NOT baked** (URDF uses `scale="0.001 0.001 0.001"` for mm→m conversion)
- Visual/Collision split: **YES**
- Visual quality: LinearDeflection=0.25mm, AngularDeflection=15°
- Collision quality: LinearDeflection=1.0mm, AngularDeflection=25°
- Volume filter: **500 mm³** (minimum, to exclude fasteners)

---

## Summary

- **Links processed:** 9 / 11 (world and tcp are virtual links)
- **STL generation:** ✅ **SUCCESS (9/9)**
- **URDF patching:** ✅ **SUCCESS**
- **BOX → mesh conversion:** ✅ **3 links** (gripper_base, gripper_left_finger, gripper_right_finger)
- **Issues:** None major, see Link Table for details

---

## Link Table

| Link | Visual STL | Collision STL | Sub-parts | Visual Size | Collision Size | Status |
|------|------------|---------------|-----------|-------------|----------------|--------|
| world | - | - | 0 | - | - | VIRTUAL (fixed) |
| base_link | visual/base_link.stl | collision/base_link.stl | 5 | 287 KB | 224 KB | ✅ OK |
| link_1 | visual/link_1.stl | collision/link_1.stl | 2 | 171 KB | 146 KB | ✅ OK |
| link_2 | visual/link_2.stl | collision/link_2.stl | 2 | 91 KB | 72 KB | ✅ OK |
| link_3 | visual/link_3.stl | collision/link_3.stl | 3 | 174 KB | 146 KB | ✅ OK |
| link_4 | visual/link_4.stl | collision/link_4.stl | 1 | 66 KB | 56 KB | ✅ OK |
| link_5 | visual/link_5.stl | collision/link_5.stl | 1 | 28 KB | 24 KB | ✅ OK (Volume=951mm³, below original 1000 threshold) |
| gripper_base | visual/gripper_base.stl | collision/gripper_base.stl | 3 | 188 KB | 156 KB | ✅ OK (merged 底座+活动侧+固定侧) |
| gripper_left_finger | visual/gripper_left_finger.stl | collision/gripper_left_finger.stl | 1 | 89 KB | 75 KB | ✅ OK (was BOX, now mesh) |
| gripper_right_finger | visual/gripper_right_finger.stl | collision/gripper_right_finger.stl | 1 | 62 KB | 50 KB | ✅ OK (was BOX, now mesh) |
| tcp | - | - | 0 | - | - | VIRTUAL (tool center point) |

**Total Visual STL files:** 9  
**Total Collision STL files:** 9  
**Total sub-parts merged:** 18

---

## URDF Changes

### Visual Updates
- **9 links** updated with new `<visual><geometry><mesh filename="../visual/<link>.stl" scale="0.001 0.001 0.001"/></geometry></visual>`
- **BOX primitives removed** from 3 gripper links

### Collision Retention
- **Existing collision primitives** (box/cylinder) **retained** for performance
- Can optionally replace with `collision/*.stl` if more accuracy needed

### Scale Consistency
- All STL meshes use `scale="0.001 0.001 0.001"` (FreeCAD mm → URDF meter)
- **Consistent across all links**

---

## Isaac Sim Import Checklist

### Pre-Import (Manual GUI Test)
1. **Launch Isaac Sim:**
   ```bash
   ~/isaac-sim.sh
   ```

2. **Import URDF:**
   - File → Import → URDF
   - Browse to: `/home/roarm_m3/roarm_isaac_clean/assets/roarm_m3/urdf/roarm_m3_v2_complete.patched.urdf`
   - Settings:
     - ☑ Fix Base: **TRUE**
     - ☐ Merge Fixed Joints: **FALSE** (debugging mode)
     - ☑ Import Inertia Tensor: **TRUE**
     - ☐ Self-Collision: **FALSE** (initial test)
     - Drive Type: **Velocity**

3. **Expected Results:**
   - **ArticulationRoot:** 1 (at `/World/roarm_m3`)
   - **RigidBodies:** ~7-9 (depends on merge)
   - **Joints:** 7 (joint_1-5, gripper_left, gripper_right)

### Visual Inspection
- [ ] All links show **complete assembly** (not single parts)
- [ ] Grippers show **actual mesh shapes** (not BOX primitives)
- [ ] No missing visual elements
- [ ] Meshes properly aligned with link origins
- [ ] No visual artifacts (flipped normals, holes)

### Joint Verification
| Joint | Type | Axis | Lower Limit | Upper Limit | Status |
|-------|------|------|-------------|-------------|--------|
| joint_1 | continuous | (0,0,1) | - | - | ⬜ Test |
| joint_2 | revolute | (0,1,0) | -1.57 | 1.57 | ⬜ Test |
| joint_3 | revolute | (0,1,0) | -1.57 | 1.57 | ⬜ Test |
| joint_4 | revolute | (0,1,0) | -1.57 | 1.57 | ⬜ Test |
| joint_5 | revolute | (0,1,0) | -1.57 | 1.57 | ⬜ Test |
| gripper_left | prismatic | (0,1,0) | 0 | 0.02 | ⬜ Test |
| gripper_right | prismatic | (0,-1,0) | 0 | 0.02 | ⬜ Test |

### Smoke Test (Manual Joint Actuation)
1. **Select robot prim** in Stage tree
2. **Open Property panel** → Physics → Articulation
3. **For each joint:**
   - Adjust target position/velocity
   - Observe: No jitter, no tunneling, smooth motion
   - Check: Visual mesh follows joint motion correctly

### Physics Stability
- [ ] Robot stable in initial pose (no immediate collapse/drift)
- [ ] No tunneling through ground plane
- [ ] Inertia values reasonable (check Property panel)
- [ ] Drive gains appropriate (no oscillation)

---

## Known Issues & Resolutions

### Issue 1: link_5 Small Volume (951 mm³)
**Problem:** Original MIN_VOLUME=1000 filtered out link_5  
**Resolution:** ✅ Lowered to MIN_VOLUME=500, now included  
**Impact:** None, link_5 STL generated successfully

### Issue 2: Gripper parts initially merged into gripper_base
**Problem:** LINK_PARTS mapping initially combined all 3 gripper parts into gripper_base  
**Resolution:** ✅ Split into 3 separate mappings:
- gripper_base: ["AL-轻量化夹头-底座"]
- gripper_left_finger: ["AL-轻量化夹头-活动侧"]
- gripper_right_finger: ["AL-轻量化夹头-固定侧"]  
**Impact:** Gripper now has proper 3-part structure matching URDF

### Issue 3: FreeCAD Chinese part names
**Problem:** Part labels contain Chinese characters, version suffixes (v1, v2)  
**Resolution:** ✅ LINK_PARTS dictionary uses multiple search terms per link  
**Impact:** All parts successfully matched

---

## Actions & Next Steps

### Immediate (Post-Import)
1. **Visual Verification:** Confirm all 9 links show complete geometry
2. **BOX Check:** Verify grippers are **NOT** boxes, but actual STL shapes
3. **Screenshot:** Capture initial import state for documentation

### Short-Term (Smoke Test)
1. **Manual Joint Actuation:** Test each joint_1-5 through full range
2. **Gripper Test:** Open/close gripper fingers
3. **Stability Check:** Let physics run for 10 seconds, observe drift/jitter

### Medium-Term (Physics Tuning)
1. **Inertia Recalculation:** If unstable, use Isaac Utils → Compute Inertia
2. **Drive Gain Tuning:** Adjust damping/stiffness if oscillation occurs
3. **Self-Collision:** Enable for gripper-only if needed

### Long-Term (Production Setup)
1. **Merge Fixed Joints:** Re-import with Merge=TRUE for performance
2. **USD Export:** Save optimized USD asset
3. **RL Integration:** Test with IsaacGym/Orbit environments

---

## Validation Criteria (Pass/Fail)

### ✅ PASS Criteria
- [x] 9/9 links have STL meshes generated
- [x] 3/3 gripper links converted from BOX to mesh
- [x] URDF patched with correct paths and scale
- [ ] ArticulationRoot detected in Isaac Sim *(requires manual GUI test)*
- [ ] All joints within expected limits *(requires manual GUI test)*
- [ ] No visual artifacts or missing geometry *(requires manual GUI test)*
- [ ] Initial pose stable (no collapse) *(requires manual GUI test)*

### ❌ FAIL Criteria (None detected so far)
- ~~Links missing visual meshes~~ (all OK)
- ~~Grippers still showing BOX~~ (patched to mesh)
- ~~URDF scale inconsistency~~ (all 0.001 0.001 0.001)
- ~~STL generation failures~~ (9/9 success)

---

## Appendix

### File Locations
- **Original URDF:** `assets/roarm_m3/urdf/roarm_m3_v2_complete.urdf`
- **Patched URDF:** `assets/roarm_m3/urdf/roarm_m3_v2_complete.patched.urdf`
- **Visual STLs:** `assets/roarm_m3/meshes/visual/*.stl` (9 files)
- **Collision STLs:** `assets/roarm_m3/meshes/collision/*.stl` (9 files)
- **Manifest:** `assets/roarm_m3/meshes/export_by_link_manifest.csv`
- **FreeCAD Parts List:** `assets/roarm_m3/meshes/visual/freecad_parts_list.csv` (5542 entries)

### Scripts Used
1. `scripts/export_stl_by_link_headless.py` - FreeCAD batch STL export
2. `scripts/patch_urdf_with_merged_stls.py` - URDF patching (BOX→mesh, paths)
3. `scripts/extract_freecad_parts_list.py` - Full parts inventory extraction

### Command History
```bash
# STL Generation
freecad --console < scripts/export_stl_by_link_headless.py

# URDF Patching
python3 scripts/patch_urdf_with_merged_stls.py

# Verification (manual)
~/isaac-sim.sh
# File → Import → URDF → roarm_m3_v2_complete.patched.urdf
```

---

**Report Generated:** 2025-10-18 16:10 KST  
**Author:** RoArm-M3 Integration Agent  
**Status:** ✅ STL Generation & URDF Patching COMPLETE | ⏳ Isaac Sim Manual Verification PENDING
