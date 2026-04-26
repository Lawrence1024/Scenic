from scenic.core.vectors import Vector
import math

from ..geometry.frame_calibration import rd_to_xodr
from ..geometry.params import get_map_path

EGO_BASE_PATH = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant"
EGO_PATH_X   = f"{EGO_BASE_PATH}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"
EGO_PATH_Y   = f"{EGO_BASE_PATH}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1"
EGO_PATH_Z   = f"{EGO_BASE_PATH}/Positions/Pos_z_Vehicle_CoorSys_E[m]/Out1"
EGO_PATH_YAW = f"{EGO_BASE_PATH}/Positions/Angle_Yaw_Vehicle_CoorSys_E[deg]/Out1"
EGO_PATH_VX  = f"{EGO_BASE_PATH}/Velocities/v_x_Vehicle_CoG[km|h]/Out1"
EGO_PATH_VY  = f"{EGO_BASE_PATH}/Velocities/v_y_Vehicle_CoG[km|h]/Out1"

EGO_READ_PATHS = (
    EGO_PATH_X, EGO_PATH_Y, EGO_PATH_Z, EGO_PATH_YAW, EGO_PATH_VX, EGO_PATH_VY
)

# GNSS (GPS) paths for ego - Environment/Road/PlantModel/GPS_POSITION/GPS_CALC
EGO_GPS_BASE = "Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC"
EGO_GPS_LONGITUDE_DEG = f"{EGO_GPS_BASE}/Longitude_deg"
EGO_GPS_LATITUDE_DEG = f"{EGO_GPS_BASE}/Latitude_deg"
EGO_GPS_HEADING_DEG = f"{EGO_GPS_BASE}/Heading_deg"
EGO_GPS_READ_PATHS = (EGO_GPS_LONGITUDE_DEG, EGO_GPS_LATITUDE_DEG, EGO_GPS_HEADING_DEG)

# Object_Sensor_3D: distance to first classified object (evaluation / ground-truth logging only — do not use for control).
EVAL_GT_DIST_OBJECT_1_M_PATH = (
    "Platform()://ASM_Traffic/Model Root/Environment/Sensors/UserInterface/DISP_Plant/"
    "Object_Sensor_3D/IdxSelect_3DSensor/Dist_Object_1[m]/Out1"
)

# GNSS (GPS) paths for Fellows - VehicleSensors/ground_truth/GPS_POSITION/GPS_CALC (indexed [i]). Use VesiInterface or Vesilnterface to match your ASM_Traffic model.
FELLOW_GPS_BASE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VehicleSensors/ground_truth/GPS_POSITION/GPS_CALC"
FELLOW_GPS_BASE_ALT = "Platform()://ASM_Traffic/Model Root/Vesilnterface/VehicleSensors/ground_truth/GPS_POSITION/GPS_CALC"

# Fellow benchmark harness: emit [FellowHarness] every N sim steps when scene.params.fellowHarnessLog is true.
_FELLOW_HARNESS_LOG_EVERY_STEPS = 50


def _maybe_log_fellow_harness(sim, fellow_index: int, speed_mps: float, x: float, y: float) -> None:
    """Periodic ``[FellowHarness]`` line when ``scene.params['fellowHarnessLog']`` is true."""
    params = getattr(getattr(sim, "scene", None), "params", None) or {}
    if not (params.get("fellowHarnessLog") or params.get("fellow_harness_log")):
        return
    ct = int(getattr(sim, "currentTime", 0) or 0)
    if ct <= 0 or ct % _FELLOW_HARNESS_LOG_EVERY_STEPS != 0:
        return
    step_s = 0.01
    try:
        ts = params.get("time_step")
        if ts is not None:
            step_s = float(ts)
    except (TypeError, ValueError):
        pass
    sim_t = ct * step_s
    print(
        f"[FellowHarness] t={sim_t:.2f}s idx={int(fellow_index)} speed_mps={float(speed_mps):.3f} "
        f"x={float(x):.3f} y={float(y):.3f}"
    )


def read_eval_gt_dist_object_1_m(sim):
    """Read dSPACE ``Dist_Object_1`` (m) from Object_Sensor_3D.

    Intended for **offline evaluation and log comparison** against Scenic center-to-center
    opponent distance. Must **not** be used in planner, MPC, shield, or any control path.
    Returns ``None`` if the simulator has no variable backend or read fails.
    """
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if not var:
        return None
    try:
        v = var.get_var(EVAL_GT_DIST_OBJECT_1_M_PATH)
        if v is None:
            return None
        return float(v)
    except Exception:
        return None


def read_ego_gps(sim):
    """Read ego GNSS (Longitude_deg, Latitude_deg, Heading_deg) from GPS_CALC. Returns (lon_deg, lat_deg, heading_deg) or (None, None, None) on failure."""
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if not var:
        return (None, None, None)
    try:
        if hasattr(var, "get_vars"):
            lon, lat, hdg = var.get_vars(EGO_GPS_READ_PATHS)
        else:
            lon = float(var.get_var(EGO_GPS_LONGITUDE_DEG))
            lat = float(var.get_var(EGO_GPS_LATITUDE_DEG))
            hdg = float(var.get_var(EGO_GPS_HEADING_DEG))
        return (float(lon), float(lat), float(hdg))
    except Exception:
        return (None, None, None)


def read_fellow_gps(sim, var, eff_index: int):
    """Read Fellow GNSS (Longitude_deg, Latitude_deg, Heading_deg) from VehicleSensors/ground_truth GPS_CALC.
    Tries FELLOW_GPS_BASE then FELLOW_GPS_BASE_ALT (VesiInterface vs Vesilnterface). Uses array variables
    when available, else indexed path. Returns (lon_deg, lat_deg, heading_deg) or (None, None, None) on failure."""
    for base in (FELLOW_GPS_BASE, FELLOW_GPS_BASE_ALT):
        try:
            lon_arr = var.get_var(f"{base}/Longitude_deg")
            lat_arr = var.get_var(f"{base}/Latitude_deg")
            hdg_arr = var.get_var(f"{base}/Heading_deg")
            if (
                isinstance(lon_arr, (list, tuple))
                and isinstance(lat_arr, (list, tuple))
                and isinstance(hdg_arr, (list, tuple))
                and 0 <= eff_index < len(lon_arr)
                and 0 <= eff_index < len(lat_arr)
                and 0 <= eff_index < len(hdg_arr)
            ):
                lon = lon_arr[eff_index]
                lat = lat_arr[eff_index]
                hdg = hdg_arr[eff_index]
                if lon is not None and lat is not None and hdg is not None:
                    return (float(lon), float(lat), float(hdg))
        except Exception:
            continue
        try:
            lon = float(var.get_var(f"{base}/Longitude_deg[{eff_index}]"))
            lat = float(var.get_var(f"{base}/Latitude_deg[{eff_index}]"))
            hdg = float(var.get_var(f"{base}/Heading_deg[{eff_index}]"))
            return (lon, lat, hdg)
        except Exception:
            continue
    return (None, None, None)


def _read_ego_state_gnss(sim, obj, var):
    """Read ego state from GNSS (position, heading) and dSPACE (z, velocity). Racing library transform converts GNSS -> Scenic local."""
    from pathlib import Path
    try:
        lon_deg, lat_deg, heading_gnss_deg = read_ego_gps(sim)
        if lon_deg is None or lat_deg is None:
            return False
        params = getattr(getattr(sim, "scene", None), "params", None) or {}
        cal_path = params.get("gnss_calibration_path")
        if not cal_path:
            _dspace = Path(__file__).resolve().parent.parent
            cal_path = _dspace / "geometry" / "gps_dspace_calibration.json"
        else:
            cal_path = Path(cal_path)
            if not cal_path.is_absolute():
                cal_path = Path.cwd() / cal_path
        if not cal_path.exists():
            print("[Ego Readback] use_gnss_readback=True but no GNSS calibration found at", cal_path)
            return False
        cal = getattr(sim, "_gnss_calibration", None)
        if cal is None:
            from scenic.domains.racing.gnss_transform import load_calibration
            cal = load_calibration(cal_path)
            sim._gnss_calibration = cal
        x, y = cal.gnss_to_local(lon_deg, lat_deg)
        yaw_rad = math.radians(float(heading_gnss_deg))
        yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
        z = 0.0
        vx_kmh = vy_kmh = 0.0
        if hasattr(var, "get_vars"):
            z = float(var.get_var(EGO_PATH_Z))
            vx_kmh = float(var.get_var(EGO_PATH_VX))
            vy_kmh = float(var.get_var(EGO_PATH_VY))
        else:
            try:
                z = float(var.get_var(EGO_PATH_Z))
                vx_kmh = float(var.get_var(EGO_PATH_VX))
                vy_kmh = float(var.get_var(EGO_PATH_VY))
            except Exception:
                pass
        vx_ms = vx_kmh / 3.6
        vy_ms = vy_kmh / 3.6
        obj.dspaceActor.position = Vector(x, y, z)
        obj.dspaceActor.heading = yaw_rad
        obj.dspaceActor.linvel = Vector(vx_ms, vy_ms, 0)
        if not getattr(sim, "_ego_read_gnss_logged", False):
            sim._ego_read_gnss_logged = True  # GNSS calibration log disabled (data already have been collected)
        return True
    except Exception as e:
        print(f"[Ego Readback] GNSS path error: {e}")
        return False


def _read_fellow_state_gnss(sim, obj, var, eff_index: int, fellow_index: int, dutils):
    """Read Fellow state from GNSS (VehicleSensors/ground_truth GPS) and FellowTrailer (z, v, w). Racing library transform converts GNSS -> Scenic local."""
    from pathlib import Path
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    try:
        lon_deg, lat_deg, heading_gnss_deg = read_fellow_gps(sim, var, eff_index)
        if lon_deg is None or lat_deg is None:
            return False
        params = getattr(getattr(sim, "scene", None), "params", None) or {}
        cal_path = params.get("gnss_calibration_path")
        if not cal_path:
            _dspace = Path(__file__).resolve().parent.parent
            cal_path = _dspace / "geometry" / "gps_dspace_calibration.json"
        else:
            cal_path = Path(cal_path)
            if not cal_path.is_absolute():
                cal_path = Path.cwd() / cal_path
        if not cal_path.exists():
            return False
        cal = getattr(sim, "_gnss_calibration", None)
        if cal is None:
            from scenic.domains.racing.gnss_transform import load_calibration
            cal = load_calibration(cal_path)
            sim._gnss_calibration = cal
        x, y = cal.gnss_to_local(lon_deg, lat_deg)
        yaw_rad = math.radians(float(heading_gnss_deg))
        yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
        z = 0.0
        v = 0.0
        w = 0.0
        try:
            z_arr = var.get_var(f"{base_path}/z")
            if isinstance(z_arr, (list, tuple)) and eff_index < len(z_arr):
                z = z_arr[eff_index] if z_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        try:
            v_arr = var.get_var(f"{base_path}/v_Fellows")
            if isinstance(v_arr, (list, tuple)) and eff_index < len(v_arr):
                v = v_arr[eff_index] if v_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        try:
            w_arr = var.get_var(f"{base_path}/w_Fellows")
            if isinstance(w_arr, (list, tuple)) and eff_index < len(w_arr):
                w = w_arr[eff_index] if w_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        # dSPACE fellow plant speed readback is in km/h; normalize to m/s for Scenic state/logging.
        v_mps = float(v) / 3.6
        obj.dspaceActor.position = Vector(x, y, float(z))
        obj.dspaceActor.heading = yaw_rad
        obj.dspaceActor.linvel = Vector(v_mps * math.cos(yaw_rad), v_mps * math.sin(yaw_rad), 0)
        obj.dspaceActor.angvel = Vector(0, 0, float(w))
        if not getattr(sim, "_fellow_read_gnss_logged", False):
            sim._fellow_read_gnss_logged = True  # GNSS calibration log disabled (data already have been collected)
        _maybe_log_fellow_harness(sim, fellow_index, v_mps, float(x), float(y))
        return True
    except Exception as e:
        print(f"[Fellow Readback] GNSS path error: {e}")
        return False


def read_ego_state(sim, obj):
    """Read ego vehicle state from variable access (MAPort or ControlDesk) into obj.dspaceActor.
    When scene params use_gnss_readback=True, reads GNSS (lon, lat, heading) and converts to Scenic
    local via the racing library GNSS transform; z and velocity still read from dSPACE.
    """
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if not var:
        return False

    params = getattr(getattr(sim, "scene", None), "params", None) or {}
    use_gnss_readback = params.get("use_gnss_readback", False)

    if use_gnss_readback:
        return _read_ego_state_gnss(sim, obj, var)

    try:
        if hasattr(var, "get_vars"):
            if not getattr(sim, "_ego_read_get_vars_logged", False):
                print("[EgoRead] using get_vars path (MAPort-compatible)")
                sim._ego_read_get_vars_logged = True
            x, y, z, yaw_deg, vx_kmh, vy_kmh = var.get_vars(EGO_READ_PATHS)
        else:
            x = float(var.get_var(EGO_PATH_X))
            y = float(var.get_var(EGO_PATH_Y))
            z = float(var.get_var(EGO_PATH_Z))
            yaw_deg = float(var.get_var(EGO_PATH_YAW))
            vx_kmh = float(var.get_var(EGO_PATH_VX))
            vy_kmh = float(var.get_var(EGO_PATH_VY))
        
        # Position from dSPACE is in RD frame. Translate to Scenic XODR frame so the
        # --2d viewer (which renders against ``param map`` in XODR-xy) shows ego on
        # the correct racing line. Identity translation if no calibration JSON exists
        # for the loaded XODR (e.g. the OLD ``LagunaSeca.xodr`` workflow).
        # See ``docs/frames.md`` and ``geometry/frame_calibration.py``.
        setattr(obj.dspaceActor, "rd_position", (float(x), float(y)))
        _xodr_path = get_map_path(getattr(getattr(sim, "scene", None), "params", None) or {})
        x_xodr, y_xodr = rd_to_xodr(float(x), float(y), _xodr_path)

        # Diagnostic: also stash the dSPACE-side GPS reading on the actor. BoundsCheck
        # uses this to compute the GPS-derived expected XODR position via pyproj and
        # compare against the translation-based position. Per-lap calibration sanity
        # check at no extra control-loop cost (one extra batched MAPort read per step).
        # Surface first failure visibly so the diagnostic stays useful (don't swallow).
        try:
            if hasattr(var, "get_vars"):
                _gps_pair = var.get_vars((EGO_GPS_LONGITUDE_DEG, EGO_GPS_LATITUDE_DEG))
                _lon, _lat = _gps_pair
            else:
                _lon = float(var.get_var(EGO_GPS_LONGITUDE_DEG))
                _lat = float(var.get_var(EGO_GPS_LATITUDE_DEG))
            setattr(obj.dspaceActor, "gps_lonlat", (float(_lon), float(_lat)))
            if not getattr(sim, "_gps_read_logged_ok", False):
                print(f"[EgoRead] GPS_CALC OK: lon={float(_lon):.6f} lat={float(_lat):.6f}")
                sim._gps_read_logged_ok = True
        except Exception as _gps_e:
            setattr(obj.dspaceActor, "gps_lonlat", None)
            if not getattr(sim, "_gps_read_logged_fail", False):
                print(f"[EgoRead] GPS_CALC read FAILED (first occurrence): "
                      f"{type(_gps_e).__name__}: {_gps_e} | "
                      f"path1={EGO_GPS_LONGITUDE_DEG} path2={EGO_GPS_LATITUDE_DEG}")
                sim._gps_read_logged_fail = True

        # yaw_deg, vx_kmh, vy_kmh already read above (get_vars or get_var)
        yaw_rad_raw = yaw_deg * (math.pi / 180.0)

        # Convert raw yaw to Scenic heading.
        #
        # IMPORTANT: Do NOT apply any 90deg or 180deg offset here unless empirically proven.
        # The raw ControlDesk yaw (Angle_Yaw_Vehicle_CoorSys_E) is already consistent with
        # the XODR geometry used by waypoints in our current setup. (Yaw convention is
        # frame-invariant under pure XY translation.)
        #
        # Normalize to [-pi, pi]
        yaw_rad = math.atan2(math.sin(yaw_rad_raw), math.cos(yaw_rad_raw))

        # Inputs are km/h, Scenic uses m/s
        vx_ms = vx_kmh / 3.6
        vy_ms = vy_kmh / 3.6

        # 4. Update Actor
        obj.dspaceActor.position = Vector(x_xodr, y_xodr, z)
        obj.dspaceActor.heading = yaw_rad
        obj.dspaceActor.linvel = Vector(vx_ms, vy_ms, 0)

        return True

    except Exception as e:
        print(f"[Ego Readback] Error: {e}")
        return False


def read_fellow_state(sim, obj, dutils):
    """Read fellow vehicle state from variable access (MAPort or ControlDesk) arrays into obj.dspaceActor."""
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if not var:
        return False
    
    # Ensure arrays are initialized (warm-up should have completed during setup)
    from .arrays import ensure_fellow_arrays_initialized
    ensure_fellow_arrays_initialized(sim)
    
    # If arrays still not ready, this is a real problem - warm-up should have fixed this
    if not sim._fellow_arrays_initialized:
        return False
    
    try:
        fellow_index = sim._getFellowIndex(obj)
        if fellow_index is None:
            return False
        eff_index = fellow_index + (sim._fellow_index_base or 0)
        params = getattr(getattr(sim, "scene", None), "params", None) or {}
        use_gnss_readback = params.get("use_gnss_readback", False)
        if use_gnss_readback:
            return _read_fellow_state_gnss(sim, obj, var, eff_index, fellow_index, dutils)
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        try:
            x_arr = var.get_var(f"{base_path}/x")
            x = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(x_arr) else 0.0
        except Exception:
            x = 0.0
        try:
            y_arr = var.get_var(f"{base_path}/y")
            y = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(y_arr) else 0.0
        except Exception:
            y = 0.0
        try:
            z_arr = var.get_var(f"{base_path}/z")
            z = z_arr[eff_index] if isinstance(z_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(z_arr) else 0.0
        except Exception:
            z = 0.0
        try:
            yaw_arr = var.get_var(f"{base_path}/yaw_deg_out")
            yaw_deg = yaw_arr[eff_index] if isinstance(yaw_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(yaw_arr) else 0.0
        except Exception:
            yaw_deg = 0.0
        v = 0.0
        w = 0.0
        try:
            v_arr = var.get_var(f"{base_path}/v_Fellows")
            if isinstance(v_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(v_arr):
                v = v_arr[eff_index] if v_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        try:
            w_arr = var.get_var(f"{base_path}/w_Fellows")
            if isinstance(w_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(w_arr):
                w = w_arr[eff_index] if w_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        if hasattr(obj, '_array_bounds_warning_shown'):
            delattr(obj, '_array_bounds_warning_shown')
        
        # Position from dSPACE is in RD frame. Translate to Scenic XODR frame for visualization
        # (identity if no calibration JSON for the loaded map). See docs/frames.md.
        setattr(obj.dspaceActor, "rd_position", (float(x), float(y)))
        _xodr_path = get_map_path(params)
        scenic_x, scenic_y = rd_to_xodr(float(x), float(y), _xodr_path)
        obj.dspaceActor.position = Vector(scenic_x, scenic_y, float(z))
        
        # Convert heading from degrees to radians (same convention as ego plant yaw; no +pi)
        yaw_rad_raw = float(yaw_deg) * (math.pi / 180.0)
        yaw_rad = math.atan2(math.sin(yaw_rad_raw), math.cos(yaw_rad_raw))
        
        # dSPACE fellow plant speed readback is in km/h; normalize to m/s for Scenic state/logging.
        v_mps = float(v) / 3.6
        obj.dspaceActor.heading = yaw_rad
        obj.dspaceActor.linvel = Vector(
            v_mps * math.cos(yaw_rad),
            v_mps * math.sin(yaw_rad),
            0
        )
        obj.dspaceActor.angvel = Vector(0, 0, float(w))
        _maybe_log_fellow_harness(sim, fellow_index, v_mps, scenic_x, scenic_y)
        return True
    except Exception as e:
        msg = str(e)
        if "bounds" in msg:
            return False
        return False

