"""Reference trajectory builder for MPC.

Builds reference heading and curvature profiles from waypoint lists
for MPC horizon prediction. Uses spline fitting with arc-length parameterization
for smooth, continuous trajectories.
"""

import numpy as np
from typing import List, Tuple, Optional, Union
import math
from scipy.interpolate import splprep, splev, UnivariateSpline


class ReferenceBuilder:
    """Builds reference trajectories from waypoints for MPC.
    
    Takes a list of waypoints and builds reference heading (psi_ref)
    and curvature (kappa_ref) arrays for the MPC prediction horizon.
    """
    
    def __init__(self, resample_dist: float = 0.2, curvature_smoothing_num: int = 15, use_splines: bool = True):
        """Initialize reference builder.
        
        Args:
            resample_dist: Distance between resampled waypoints (meters)
            curvature_smoothing_num: Number of points apart for curvature calculation (smoothing)
            use_splines: If True, use spline fitting with arc-length parameterization (default: True)
        """
        self.resample_dist = resample_dist
        self.curvature_smoothing_num = curvature_smoothing_num
        self.use_splines = use_splines
        self._last_nearest_idx = 0
        self._spline_cache = None  # Cache for spline parameters
        self._spline_waypoints = None  # Cache for waypoints used to build spline
        self._current_s_0 = 0.0  # Current arc-length progress (meters)
        self._current_u_0 = 0.0  # Current spline parameter value
    
    def _is_3d_waypoint(self, waypoint) -> bool:
        """Check if waypoint is 3D (has z coordinate)."""
        return len(waypoint) >= 3
    
    def _is_3d_waypoints(self, waypoints) -> bool:
        """Check if waypoints list contains 3D points."""
        if not waypoints or len(waypoints) == 0:
            return False
        return self._is_3d_waypoint(waypoints[0])
    
    def find_nearest_waypoint(self, 
                              position: Tuple[float, float],
                              waypoints: List[Tuple[float, float]],
                              last_idx: Optional[int] = None,
                              adaptive_search: bool = True,
                              cte_magnitude: Optional[float] = None) -> int:
        """Find nearest waypoint index to current position.
        
        Uses forward-only search starting from last_idx to ensure
        forward progress along the path. Search window is adaptive
        based on CTE magnitude when vehicle is far off-track.
        
        Supports both 2D (x, y) and 3D (x, y, z) waypoints.
        
        Args:
            position: Current vehicle position (x, y) or (x, y, z)
            waypoints: List of waypoint (x, y) or (x, y, z) tuples
            last_idx: Last known waypoint index (for forward-only search)
            adaptive_search: If True, adapt search window based on CTE
            cte_magnitude: Current CTE magnitude (meters) for adaptive search
            
        Returns:
            Index of nearest waypoint
        """
        if not waypoints or len(waypoints) == 0:
            return 0
        
        # Handle both 2D and 3D positions
        px, py = position[0], position[1]
        pz = position[2] if len(position) >= 3 else 0.0
        is_3d = self._is_3d_waypoints(waypoints)
        
        start_idx = last_idx if last_idx is not None else self._last_nearest_idx
        
        # Adaptive search window: scale with CTE magnitude
        # Base window: 200 waypoints (~40m at 0.2m spacing)
        # When CTE is large, expand search window AND increase lookback
        base_forward = 200  # Increased from 50
        base_lookback = 20  # Base lookback when on-track
        
        if adaptive_search and cte_magnitude is not None:
            # Scale search window based on CTE
            # At 0m CTE: base window
            # At 10m CTE: 2x window
            # At 20m+ CTE: 3x window
            if cte_magnitude >= 20.0:
                scale = 3.0
            elif cte_magnitude >= 10.0:
                scale = 2.0
            elif cte_magnitude >= 5.0:
                scale = 1.5
            else:
                scale = 1.0
            
            forward_window = int(base_forward * scale)
            
            # CRITICAL FIX: Increase lookback significantly when CTE is large
            # When vehicle is far off-track, it might be behind the last known waypoint
            # Need more aggressive lookback to find the actual nearest waypoint
            if cte_magnitude >= 5.0:
                # When far off-track, use much larger lookback (up to 50% of forward window)
                lookback_window = int(base_forward * scale * 0.5)  # 50% of forward window
            else:
                # Normal case: small lookback for safety
                lookback_window = int(base_lookback * scale)
        else:
            forward_window = base_forward
            lookback_window = base_lookback
        
        # Search from last index with adaptive lookback
        # More lookback when off-track to find actual nearest waypoint
        search_start = max(0, start_idx - lookback_window)
        search_end = min(len(waypoints), start_idx + forward_window)
        
        best_idx = start_idx
        best_dist2 = float('inf')
        
        for i in range(search_start, search_end):
            wp = waypoints[i]
            wx, wy = wp[0], wp[1]
            wz = wp[2] if len(wp) >= 3 else 0.0
            
            dx = px - wx
            dy = py - wy
            if is_3d:
                dz = pz - wz
                dist2 = dx*dx + dy*dy + dz*dz
            else:
                dist2 = dx*dx + dy*dy
            
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_idx = i
        
        self._last_nearest_idx = best_idx
        return best_idx
    
    def _fit_spline(self, waypoints: List[Tuple[float, float]]) -> Tuple:
        """Fit a parametric spline through waypoints.
        
        Supports both 2D (x, y) and 3D (x, y, z) waypoints.
        
        Args:
            waypoints: List of waypoint (x, y) or (x, y, z) tuples
            
        Returns:
            Tuple of (tck, u) where tck is spline representation and u is parameter array
        """
        if len(waypoints) < 2:
            return None
        
        # Check if waypoints are 3D
        is_3d = self._is_3d_waypoints(waypoints)
        
        # Convert to numpy arrays
        waypoints_array = np.array(waypoints, dtype=np.float64)
        x = waypoints_array[:, 0]
        y = waypoints_array[:, 1]
        
        # Fit parametric spline (s=0 means no smoothing, just interpolation)
        # k=3 for cubic splines (smooth curvature)
        try:
            if is_3d:
                z = waypoints_array[:, 2]
                tck, u = splprep([x, y, z], s=0, k=min(3, len(waypoints)-1), per=0)
            else:
                tck, u = splprep([x, y], s=0, k=min(3, len(waypoints)-1), per=0)
            return (tck, u)
        except Exception as e:
            if not getattr(ReferenceBuilder, '_spline_fallback_logged', False):
                print(f"[ReferenceBuilder] Spline fitting failed: {e}, falling back to linear")
                ReferenceBuilder._spline_fallback_logged = True
            return None
    
    def _compute_arc_length_parameterization(self, tck, u_param: np.ndarray, num_points: int) -> Tuple[np.ndarray, np.ndarray]:
        """Compute arc-length parameterization of spline.
        
        Supports both 2D and 3D splines.
        
        Args:
            tck: Spline representation from splprep
            u_param: Original parameter array
            num_points: Number of points for arc-length sampling
            
        Returns:
            Tuple of (u_arc, s_arc) where:
            - u_arc: Parameter values at uniform arc-length intervals
            - s_arc: Arc-length values (cumulative distance)
        """
        # Evaluate spline at many points to compute arc-length
        u_fine = np.linspace(0, 1, max(1000, num_points * 10))
        points_fine = splev(u_fine, tck)
        
        # Compute arc-length at each point (supports both 2D and 3D)
        dx = np.diff(points_fine[0])
        dy = np.diff(points_fine[1])
        if len(points_fine) >= 3:
            # 3D case
            dz = np.diff(points_fine[2])
            ds = np.sqrt(dx*dx + dy*dy + dz*dz)
        else:
            # 2D case
            ds = np.sqrt(dx*dx + dy*dy)
        
        s_cumulative = np.concatenate(([0], np.cumsum(ds)))
        total_length = s_cumulative[-1]
        
        if total_length < 1e-6:
            # Degenerate case: return uniform parameterization
            return (np.linspace(0, 1, num_points), np.linspace(0, total_length, num_points))
        
        # Create uniform arc-length samples
        s_uniform = np.linspace(0, total_length, num_points)
        
        # Find parameter values u that correspond to these arc-lengths
        u_arc = np.interp(s_uniform, s_cumulative, u_fine)
        
        return (u_arc, s_uniform)
    
    def project_to_spline(self, 
                         position: Tuple[float, float],
                         waypoints: List[Tuple[float, float]],
                         tck=None,
                         u_param: Optional[np.ndarray] = None) -> Tuple[float, float, int]:
        """Project vehicle position onto spline to get arc-length progress.
        
        Uses iterative projection (Newton-Raphson) to find the closest point on the spline.
        
        Args:
            position: Current vehicle position (x, y) or (x, y, z)
            waypoints: List of waypoint (x, y) or (x, y, z) tuples
            tck: Optional pre-computed spline representation (if None, will fit spline)
            u_param: Optional pre-computed parameter array (if None, will compute)
            
        Returns:
            Tuple of (s_0, u_0, nearest_segment_idx) where:
            - s_0: Arc-length along spline to projected point (meters)
            - u_0: Spline parameter value at projected point [0, 1]
            - nearest_segment_idx: Index of waypoint segment containing projection
        """
        if not waypoints or len(waypoints) < 2:
            return (0.0, 0.0, 0)
        
        # Handle both 2D and 3D positions
        px, py = position[0], position[1]
        is_3d = self._is_3d_waypoints(waypoints)
        
        # Fit spline if not provided
        if tck is None:
            spline_result = self._fit_spline(waypoints)
            if spline_result is None:
                # Fallback: use distance-based method
                nearest_idx = self.find_nearest_waypoint(position, waypoints, self._last_nearest_idx)
                return (0.0, 0.0, nearest_idx)
            tck, u_param = spline_result
        
        # Use Newton-Raphson to find closest point on spline
        # Start from current progress if available, otherwise use nearest waypoint
        if self._current_u_0 > 0.0 and self._current_u_0 < 1.0:
            u_guess = self._current_u_0
        else:
            # Find nearest waypoint and use its parameter
            nearest_idx = self.find_nearest_waypoint(position, waypoints, self._last_nearest_idx)
            if nearest_idx < len(u_param):
                u_guess = float(u_param[nearest_idx])
            else:
                u_guess = 0.0
        
        # Newton-Raphson iteration to minimize distance
        max_iter = 20
        tolerance = 1e-6
        u = np.clip(u_guess, 0.0, 1.0)
        
        for _ in range(max_iter):
            # Evaluate spline and derivatives at current u
            point = splev(u, tck)
            deriv = splev(u, tck, der=1)
            deriv2 = splev(u, tck, der=2)
            
            # Compute distance vector and its derivatives
            dx = px - point[0]
            dy = py - point[1]
            if is_3d and len(position) >= 3 and len(point) >= 3:
                dz = position[2] - point[2]
                dist_sq = dx*dx + dy*dy + dz*dz
                # For 3D, project to XY plane for segment selection
                dist_sq_xy = dx*dx + dy*dy
            else:
                dist_sq = dx*dx + dy*dy
                dist_sq_xy = dist_sq
            
            # Gradient of distance^2 with respect to u (only use dz when we set it above)
            ddist_sq_du = -2.0 * (dx * deriv[0] + dy * deriv[1])
            if is_3d and len(position) >= 3 and len(point) >= 3 and len(deriv) >= 3:
                ddist_sq_du += -2.0 * dz * deriv[2]
            
            # Hessian (second derivative)
            d2dist_sq_du2 = 2.0 * (deriv[0]*deriv[0] + deriv[1]*deriv[1] - dx*deriv2[0] - dy*deriv2[1])
            if is_3d and len(position) >= 3 and len(point) >= 3 and len(deriv) >= 3 and len(deriv2) >= 3:
                d2dist_sq_du2 += 2.0 * (deriv[2]*deriv[2] - dz*deriv2[2])
            
            # Update u using Newton-Raphson
            if abs(d2dist_sq_du2) > 1e-9:
                u_new = u - ddist_sq_du / d2dist_sq_du2
                u_new = np.clip(u_new, 0.0, 1.0)
                
                # Check convergence
                if abs(u_new - u) < tolerance:
                    u = u_new
                    break
                u = u_new
            else:
                # Degenerate case: use bisection
                break
        
        # Compute arc-length s_0 at u
        u_fine = np.linspace(0, 1, 1000)
        points_fine = splev(u_fine, tck)
        dx_fine = np.diff(points_fine[0])
        dy_fine = np.diff(points_fine[1])
        if is_3d and len(points_fine) >= 3:
            dz_fine = np.diff(points_fine[2])
            ds_fine = np.sqrt(dx_fine*dx_fine + dy_fine*dy_fine + dz_fine*dz_fine)
        else:
            ds_fine = np.sqrt(dx_fine*dx_fine + dy_fine*dy_fine)
        
        s_cumulative = np.concatenate(([0], np.cumsum(ds_fine)))
        s_0 = float(np.interp(u, u_fine, s_cumulative))
        
        # Find nearest segment index
        # Map u to waypoint index
        if len(u_param) > 0:
            nearest_segment_idx = int(np.interp(u, u_param, np.arange(len(u_param))))
            nearest_segment_idx = max(0, min(nearest_segment_idx, len(waypoints) - 2))
        else:
            nearest_segment_idx = 0
        
        # Update progress tracking
        self._current_s_0 = s_0
        self._current_u_0 = float(u)
        
        return (s_0, float(u), nearest_segment_idx)
    
    def _resample_waypoints_linear(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Linear resampling of waypoints (fallback method).
        
        Supports both 2D (x, y) and 3D (x, y, z) waypoints.
        
        Args:
            waypoints: Original waypoint list
            
        Returns:
            Resampled waypoint list with uniform spacing (preserves dimensionality)
        """
        if len(waypoints) < 2:
            return waypoints
        
        is_3d = self._is_3d_waypoints(waypoints)
        resampled = [waypoints[0]]
        current_dist = 0.0
        
        for i in range(1, len(waypoints)):
            wp0 = waypoints[i-1]
            wp1 = waypoints[i]
            x0, y0 = wp0[0], wp0[1]
            x1, y1 = wp1[0], wp1[1]
            dx = x1 - x0
            dy = y1 - y0
            
            # Compute segment length (3D if available, otherwise 2D)
            if is_3d and len(wp0) >= 3 and len(wp1) >= 3:
                dz = wp1[2] - wp0[2]
                seg_len = math.sqrt(dx*dx + dy*dy + dz*dz)
            else:
                seg_len = math.sqrt(dx*dx + dy*dy)
            
            if seg_len < 1e-6:
                continue
            
            # Add points along this segment
            while current_dist + self.resample_dist < seg_len:
                current_dist += self.resample_dist
                u = current_dist / seg_len
                if is_3d and len(wp0) >= 3 and len(wp1) >= 3:
                    z0 = wp0[2]
                    z1 = wp1[2]
                    resampled.append((x0 + u*dx, y0 + u*dy, z0 + u*(z1 - z0)))
                else:
                    resampled.append((x0 + u*dx, y0 + u*dy))
            
            # Move to next segment
            current_dist = current_dist + self.resample_dist - seg_len
            if current_dist < 0:
                current_dist = 0.0
        
        # Always include last waypoint
        if resampled[-1] != waypoints[-1]:
            resampled.append(waypoints[-1])
        
        return resampled
    
    def resample_waypoints(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Resample waypoints with curvature-adaptive spacing using splines and arc-length parameterization.
        
        Uses adaptive spacing based on curvature:
        - Low curvature (|κ| < 0.01, R > 100m): sample every 1.0-2.0m
        - High curvature (|κ| ≥ 0.01): sample every 0.25-0.5m
        
        Args:
            waypoints: Original waypoint list
            
        Returns:
            Resampled waypoint list with curvature-adaptive arc-length spacing
        """
        if len(waypoints) < 2:
            return waypoints
        
        if not self.use_splines:
            return self._resample_waypoints_linear(waypoints)
        
        # Use spline-based resampling with curvature-adaptive spacing
        try:
            # Fit spline
            spline_result = self._fit_spline(waypoints)
            if spline_result is None:
                # Fallback to linear
                return self._resample_waypoints_linear(waypoints)
            
            tck, u = spline_result
            
            # Compute total arc-length and curvature profile
            u_fine = np.linspace(0, 1, 1000)
            points_fine = splev(u_fine, tck)
            deriv_fine = splev(u_fine, tck, der=1)
            deriv2_fine = splev(u_fine, tck, der=2)
            
            dx = np.diff(points_fine[0])
            dy = np.diff(points_fine[1])
            ds = np.sqrt(dx*dx + dy*dy)
            s_cumulative = np.concatenate(([0], np.cumsum(ds)))
            total_length = s_cumulative[-1]
            
            if total_length < 1e-6:
                return waypoints
            
            # Compute curvature at each fine point
            curvature = np.zeros(len(u_fine))
            for i in range(len(u_fine)):
                dx_dt = deriv_fine[0][i]
                dy_dt = deriv_fine[1][i]
                d2x_dt2 = deriv2_fine[0][i]
                d2y_dt2 = deriv2_fine[1][i]
                speed_tangent = np.sqrt(dx_dt*dx_dt + dy_dt*dy_dt)
                if speed_tangent > 1e-9:
                    numerator = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
                    denominator = speed_tangent ** 3
                    curvature[i] = numerator / denominator if denominator > 1e-9 else 0.0
            
            # Adaptive resampling: sample more densely in high-curvature regions
            resampled = []
            s_current = 0.0
            min_resample_dist = 0.25  # Minimum spacing in high curvature (0.25m)
            max_resample_dist = 2.0   # Maximum spacing in low curvature (2.0m)
            curvature_threshold = 0.01  # 1/m (R = 100m)
            
            while s_current < total_length:
                # Find curvature at current position
                u_current = np.interp(s_current, s_cumulative, u_fine)
                kappa_current = float(np.interp(u_current, u_fine, curvature))
                
                # Adaptive spacing based on curvature
                abs_kappa = abs(kappa_current)
                if abs_kappa >= curvature_threshold:
                    # High curvature: use fine spacing (0.25-0.5m)
                    resample_dist_local = min_resample_dist + (0.5 - min_resample_dist) * min(1.0, abs_kappa / 0.1)
                else:
                    # Low curvature: use coarse spacing (1.0-2.0m)
                    resample_dist_local = min_resample_dist + (max_resample_dist - min_resample_dist) * (1.0 - abs_kappa / curvature_threshold)
                
                # Evaluate spline at current arc-length
                point = splev(u_current, tck)
                if len(point) >= 3:
                    resampled.append((float(point[0]), float(point[1]), float(point[2])))
                else:
                    resampled.append((float(point[0]), float(point[1])))
                
                # Advance by adaptive spacing
                s_current += resample_dist_local
            
            # Always include last waypoint
            if len(resampled) == 0 or resampled[-1] != waypoints[-1]:
                resampled.append(waypoints[-1])
            
            return resampled
            
        except Exception as e:
            if not getattr(ReferenceBuilder, '_spline_fallback_logged', False):
                print(f"[ReferenceBuilder] Spline resampling failed: {e}, falling back to linear")
                ReferenceBuilder._spline_fallback_logged = True
            return self._resample_waypoints_linear(waypoints)
    
    def compute_curvature(self, 
                         p0: Tuple[float, float],
                         p1: Tuple[float, float],
                         p2: Tuple[float, float]) -> float:
        """Compute curvature using 3-point method.
        
        Args:
            p0: First point (x, y)
            p1: Middle point (x, y)
            p2: Third point (x, y)
            
        Returns:
            Curvature (1/radius) in 1/meters
        """
        x0, y0 = p0
        x1, y1 = p1
        x2, y2 = p2
        
        # Vectors
        v1x = x1 - x0
        v1y = y1 - y0
        v2x = x2 - x1
        v2y = y2 - y1
        
        # Cross product (z-component)
        cross = v1x * v2y - v1y * v2x
        
        # Lengths
        len1 = math.sqrt(v1x*v1x + v1y*v1y)
        len2 = math.sqrt(v2x*v2x + v2y*v2y)
        
        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0
        
        # Curvature = cross / (len1 * len2 * average_length)
        avg_len = (len1 + len2) / 2.0
        if avg_len < 1e-6:
            return 0.0
        
        curvature = 2.0 * cross / (len1 * len2 * avg_len)
        return curvature
    
    def build_reference(self,
                       waypoints: List[Tuple[float, float]],
                       current_position: Tuple[float, float],
                       current_heading: float,
                       horizon_steps: int,
                       dt: float,
                       speed: float,
                       last_waypoint_idx: Optional[int] = None,
                       cte_magnitude: Optional[float] = None,
                       v_ref_profile: Optional[Union[List[float], np.ndarray]] = None
                       ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, float, np.ndarray]:
        """Build reference trajectory for MPC horizon (Phase 1: s-based parameterization).
        
        Path is parameterized by arc length s. Current progress s_0 is computed by
        projecting the vehicle onto the path; the horizon is built as s_0, s_0+Δs_1, ...
        with Δs_k from speed (constant) or from v_ref_profile when provided (trajectory-aware).
        
        When v_ref_profile is provided, lateral and longitudinal MPC share the same speed
        plan so the controller "sees the whole trajectory" and avoids over-brake then throttle
        or over-steer then correct.
        
        Supports both 2D (x, y) and 3D (x, y, z) waypoints.
        
        Args:
            waypoints: List of waypoint (x, y) or (x, y, z) tuples
            current_position: Current vehicle position (x, y) or (x, y, z)
            current_heading: Current vehicle heading (radians)
            horizon_steps: Number of prediction steps
            dt: Time step (seconds)
            speed: Current vehicle speed (m/s) - used when v_ref_profile is None
            last_waypoint_idx: Last known waypoint index
            cte_magnitude: Optional CTE magnitude for adaptive search
            v_ref_profile: Optional speed profile over horizon (m/s). If length matches
                horizon_steps, used for v_ref and for s_horizon (s_k = s_0 + sum(v_ref[0:k+1]*dt)).
            
        Returns:
            Tuple of (psi_ref, kappa_ref, v_ref, grade_ref, new_waypoint_idx, s_0, s_horizon)
            - psi_ref: Reference heading array (radians) - yaw angle in XY plane
            - kappa_ref: Reference curvature array (1/meters) - curvature in XY plane
            - v_ref: Reference speed array (m/s) - from v_ref_profile or constant speed
            - grade_ref: Reference road grade array (radians) - pitch angle (positive = uphill)
            - new_waypoint_idx: Updated waypoint index
            - s_0: Current progress along path (arc length from path start to projected position, meters)
            - s_horizon: Arc length at each horizon step (length horizon_steps), meters
        """
        horizon_steps = int(horizon_steps)
        if not waypoints or len(waypoints) < 2:
            # Return zero references if no waypoints
            s_horizon = np.zeros(max(1, horizon_steps), dtype=np.float64)
            v_ref = np.array(v_ref_profile, dtype=np.float64) if (v_ref_profile is not None and len(v_ref_profile) == horizon_steps) else np.full(horizon_steps, speed)
            return (
                np.zeros(horizon_steps),
                np.zeros(horizon_steps),
                v_ref,
                np.zeros(horizon_steps),  # grade_ref
                0,
                0.0,   # s_0
                s_horizon
            )
        
        # Check if waypoints are 3D
        is_3d = self._is_3d_waypoints(waypoints)
        
        # Find nearest waypoint
        # Use provided CTE magnitude if available, otherwise estimate from distance to last waypoint
        cte_estimate = cte_magnitude
        if cte_estimate is None and last_waypoint_idx is not None and last_waypoint_idx < len(waypoints):
            last_wp = waypoints[last_waypoint_idx]
            dx = current_position[0] - last_wp[0]
            dy = current_position[1] - last_wp[1]
            if is_3d and len(current_position) >= 3 and len(last_wp) >= 3:
                dz = current_position[2] - last_wp[2]
                cte_estimate = (dx*dx + dy*dy + dz*dz) ** 0.5
            else:
                cte_estimate = (dx*dx + dy*dy) ** 0.5
        
        nearest_idx = self.find_nearest_waypoint(
            current_position, waypoints, last_waypoint_idx,
            adaptive_search=True, cte_magnitude=cte_estimate
        )
        
        if horizon_steps <= 0:
            raise ValueError(f"horizon_steps must be > 0, got {horizon_steps}")
        
        # Build reference arrays - ensure they are 1D numpy arrays
        psi_ref = np.zeros(horizon_steps, dtype=np.float64)
        kappa_ref = np.zeros(horizon_steps, dtype=np.float64)
        if v_ref_profile is not None and len(v_ref_profile) == horizon_steps:
            v_ref = np.asarray(v_ref_profile, dtype=np.float64)
            if v_ref.ndim != 1:
                v_ref = np.ravel(v_ref)[:horizon_steps]
            horizon_dist = float(np.sum(v_ref) * dt)
        else:
            v_ref = np.full(horizon_steps, float(speed), dtype=np.float64)
            horizon_dist = speed * horizon_steps * dt
        grade_ref = np.zeros(horizon_steps, dtype=np.float64)  # Road grade (pitch angle) in radians
        
        # Helper function to flip heading by 180° if opposite to vehicle heading
        def adjust_heading_if_opposite(seg_heading, vehicle_heading):
            """Flip segment heading by 180° if it's opposite to vehicle heading (>90° difference)."""
            heading_diff = seg_heading - vehicle_heading
            # Normalize to [-pi, pi]
            heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
            if abs(heading_diff) > math.pi / 2:  # > 90 degrees
                # Flip by 180°
                flipped = seg_heading + math.pi
                return math.atan2(math.sin(flipped), math.cos(flipped))  # Normalize to [-pi, pi]
            return seg_heading
        
        # Use spline-based reference building if enabled
        if self.use_splines and len(waypoints) >= 3:
            try:
                # Extract a window of waypoints around current position for spline fitting
                # Use enough waypoints to cover horizon distance plus some margin
                window_size = max(50, int(horizon_dist / 0.2) + 20)  # Assume ~0.2m spacing
                start_idx = max(0, nearest_idx - 10)
                end_idx = min(len(waypoints), nearest_idx + window_size)
                waypoint_window = waypoints[start_idx:end_idx]
                
                if len(waypoint_window) >= 3:
                    # Fit spline through waypoint window
                    spline_result = self._fit_spline(waypoint_window)
                    if spline_result is not None:
                        tck, u = spline_result
                        
                        # Phase 1: Current progress s_0 = arc length from path (window) start to projected position
                        s_0, u_0, _ = self.project_to_spline(
                            current_position, waypoint_window, tck=tck, u_param=u
                        )
                        
                        # Full window arc-length parameterization (u from 0 to 1)
                        u_fine = np.linspace(0.0, 1.0, 1000)
                        points_fine = splev(u_fine, tck)
                        dx = np.diff(points_fine[0])
                        dy = np.diff(points_fine[1])
                        if len(points_fine) >= 3 and is_3d:
                            dz = np.diff(points_fine[2])
                            ds = np.sqrt(dx*dx + dy*dy + dz*dz)
                        else:
                            ds = np.sqrt(dx*dx + dy*dy)
                        s_cumulative = np.concatenate(([0], np.cumsum(ds)))
                        total_length = s_cumulative[-1]
                        
                        # Horizon in s: use v_ref so lateral MPC sees same trajectory as longitudinal (smooth turns, no overshoot)
                        s_horizon = np.zeros(horizon_steps, dtype=np.float64)
                        for k in range(horizon_steps):
                            target_s = s_0 + dt * float(np.sum(v_ref[:k + 1]))
                            s_horizon[k] = min(target_s, total_length)
                            
                            if target_s >= total_length:
                                u_k = 1.0
                            else:
                                u_k = np.interp(target_s, s_cumulative, u_fine)
                            
                            # Evaluate spline and derivatives at u_k
                            point_k = splev(u_k, tck)
                            deriv_k = splev(u_k, tck, der=1)  # First derivative (tangent)
                            deriv2_k = splev(u_k, tck, der=2)  # Second derivative (for curvature)
                            
                            # Compute heading from tangent (first derivative)
                            # For 3D, project to XY plane (use only x and y components)
                            dx_dt = deriv_k[0]
                            dy_dt = deriv_k[1]
                            speed_tangent = math.sqrt(dx_dt*dx_dt + dy_dt*dy_dt)
                            
                            if speed_tangent > 1e-6:
                                seg_heading = math.atan2(dy_dt, dx_dt)
                                psi_ref[k] = adjust_heading_if_opposite(seg_heading, current_heading)
                                
                                # Compute curvature from spline derivatives
                                # For 3D, compute 2D curvature in XY plane (project to XY plane)
                                # kappa = |x'*y'' - y'*x''| / (x'^2 + y'^2)^(3/2)
                                d2x_dt2 = deriv2_k[0]
                                d2y_dt2 = deriv2_k[1]
                                numerator = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
                                denominator = speed_tangent ** 3
                                if denominator > 1e-9:
                                    kappa_ref[k] = numerator / denominator
                                else:
                                    kappa_ref[k] = 0.0
                                
                                # Compute road grade (pitch angle) from 3D spline
                                # grade = atan2(dz, sqrt(dx^2 + dy^2)) - positive = uphill
                                if is_3d and len(deriv_k) >= 3:
                                    dz_dt = deriv_k[2]
                                    speed_3d = math.sqrt(dx_dt*dx_dt + dy_dt*dy_dt + dz_dt*dz_dt)
                                    if speed_3d > 1e-6:
                                        # Grade angle: positive = uphill, negative = downhill
                                        grade_ref[k] = math.atan2(dz_dt, speed_tangent)
                                    else:
                                        grade_ref[k] = 0.0
                                else:
                                    grade_ref[k] = 0.0
                            else:
                                # Degenerate: use fallback
                                psi_ref[k] = current_heading
                                kappa_ref[k] = 0.0
                                grade_ref[k] = 0.0
                        
                        # Successfully used splines (Phase 1: return s_0 and s_horizon)
                        if len(psi_ref) != horizon_steps or len(kappa_ref) != horizon_steps or len(grade_ref) != horizon_steps:
                            raise ValueError("Spline reference arrays have wrong length")
                        return (psi_ref, kappa_ref, v_ref, grade_ref, nearest_idx, float(s_0), s_horizon)
            except Exception as e:
                if not getattr(ReferenceBuilder, '_spline_fallback_logged', False):
                    print(f"[ReferenceBuilder] Spline-based reference building failed: {e}, falling back to linear")
                    ReferenceBuilder._spline_fallback_logged = True
                # Fall through to linear method
        
        # Fallback: Linear interpolation along waypoint segments (Phase 1: s-based)
        # Precompute cumulative arc length from path start: s_cumulative_waypoints[i] = length to waypoint i
        s_cumulative_waypoints = [0.0]
        for i in range(1, len(waypoints)):
            wp0, wp1 = waypoints[i - 1], waypoints[i]
            dx = wp1[0] - wp0[0]
            dy = wp1[1] - wp0[1]
            if is_3d and len(wp0) >= 3 and len(wp1) >= 3:
                dz = wp1[2] - wp0[2]
                seg_len = math.sqrt(dx*dx + dy*dy + dz*dz)
            else:
                seg_len = math.sqrt(dx*dx + dy*dy)
            s_cumulative_waypoints.append(s_cumulative_waypoints[-1] + seg_len)
        total_length = s_cumulative_waypoints[-1]
        
        # Current progress s_0: project current position onto segment at nearest_idx
        seg_idx_0 = min(nearest_idx, len(waypoints) - 2)
        wp0 = waypoints[seg_idx_0]
        wp1 = waypoints[seg_idx_0 + 1]
        x0, y0 = wp0[0], wp0[1]
        x1, y1 = wp1[0], wp1[1]
        seg_dx, seg_dy = x1 - x0, y1 - y0
        seg_len_0 = math.sqrt(seg_dx*seg_dx + seg_dy*seg_dy) if (seg_dx or seg_dy) else 1e-9
        wx = current_position[0] - x0
        wy = current_position[1] - y0
        u_proj = np.clip((wx*seg_dx + wy*seg_dy) / (seg_len_0*seg_len_0), 0.0, 1.0)
        s_0 = s_cumulative_waypoints[seg_idx_0] + u_proj * (s_cumulative_waypoints[seg_idx_0 + 1] - s_cumulative_waypoints[seg_idx_0])
        
        s_horizon = np.zeros(horizon_steps, dtype=np.float64)
        
        for k in range(horizon_steps):
            target_s = s_0 + dt * float(np.sum(v_ref[:k + 1]))
            s_horizon[k] = min(target_s, total_length)
            target_s = s_horizon[k]
            
            # Find segment j such that s_cumulative_waypoints[j] <= target_s < s_cumulative_waypoints[j+1]
            j = 0
            while j < len(waypoints) - 1 and s_cumulative_waypoints[j + 1] <= target_s:
                j += 1
            current_idx = min(j, len(waypoints) - 2)
            
            wp0 = waypoints[current_idx]
            wp1 = waypoints[current_idx + 1]
            x0, y0 = wp0[0], wp0[1]
            x1, y1 = wp1[0], wp1[1]
            dx = x1 - x0
            dy = y1 - y0
            if is_3d and len(wp0) >= 3 and len(wp1) >= 3:
                dz = wp1[2] - wp0[2]
                seg_len = math.sqrt(dx*dx + dy*dy + dz*dz)
            else:
                seg_len = math.sqrt(dx*dx + dy*dy)
            
            if seg_len < 1e-6:
                psi_ref[k] = current_heading
                kappa_ref[k] = 0.0
                grade_ref[k] = 0.0
            else:
                u = (target_s - s_cumulative_waypoints[current_idx]) / seg_len
                u = max(0.0, min(1.0, u))
                ref_x = x0 + u * dx
                ref_y = y0 + u * dy
                
                seg_heading = math.atan2(dy, dx)
                psi_ref[k] = adjust_heading_if_opposite(seg_heading, current_heading)
                
                smoothing_offset = max(1, self.curvature_smoothing_num)
                if current_idx >= smoothing_offset and current_idx < len(waypoints) - smoothing_offset:
                    p0 = (waypoints[current_idx - smoothing_offset][0], waypoints[current_idx - smoothing_offset][1])
                    p1 = (waypoints[current_idx][0], waypoints[current_idx][1])
                    p2 = (waypoints[current_idx + smoothing_offset][0], waypoints[current_idx + smoothing_offset][1])
                    kappa_ref[k] = self.compute_curvature(p0, p1, p2)
                elif current_idx > 0 and current_idx < len(waypoints) - 1:
                    p0 = (waypoints[current_idx - 1][0], waypoints[current_idx - 1][1])
                    p1 = (waypoints[current_idx][0], waypoints[current_idx][1])
                    p2 = (waypoints[current_idx + 1][0], waypoints[current_idx + 1][1])
                    kappa_ref[k] = self.compute_curvature(p0, p1, p2)
                else:
                    kappa_ref[k] = 0.0
                
                if is_3d and len(wp0) >= 3 and len(wp1) >= 3:
                    dz = wp1[2] - wp0[2]
                    seg_len_xy = math.sqrt(dx*dx + dy*dy)
                    grade_ref[k] = math.atan2(dz, seg_len_xy) if seg_len_xy > 1e-6 else 0.0
                else:
                    grade_ref[k] = 0.0
        
        # Verify arrays have correct shape and length before returning
        # Arrays should already be correctly initialized with shape (horizon_steps,)
        # Don't modify the arrays - just validate them
        
        # Check that arrays are numpy arrays
        if not isinstance(psi_ref, np.ndarray):
            raise TypeError(f"psi_ref must be a numpy array, got {type(psi_ref)}")
        if not isinstance(kappa_ref, np.ndarray):
            raise TypeError(f"kappa_ref must be a numpy array, got {type(kappa_ref)}")
        if not isinstance(v_ref, np.ndarray):
            raise TypeError(f"v_ref must be a numpy array, got {type(v_ref)}")
        if not isinstance(grade_ref, np.ndarray):
            raise TypeError(f"grade_ref must be a numpy array, got {type(grade_ref)}")
        
        # Ensure arrays are 1D with correct length
        if psi_ref.ndim != 1:
            raise ValueError(f"psi_ref must be 1D, got {psi_ref.ndim}D array with shape {psi_ref.shape}")
        if kappa_ref.ndim != 1:
            raise ValueError(f"kappa_ref must be 1D, got {kappa_ref.ndim}D array with shape {kappa_ref.shape}")
        if v_ref.ndim != 1:
            raise ValueError(f"v_ref must be 1D, got {v_ref.ndim}D array with shape {v_ref.shape}")
        if grade_ref.ndim != 1:
            raise ValueError(f"grade_ref must be 1D, got {grade_ref.ndim}D array with shape {grade_ref.shape}")
        
        # Verify arrays have correct length - this is the critical check
        if len(psi_ref) != horizon_steps:
            raise ValueError(f"psi_ref length mismatch: expected {horizon_steps}, got {len(psi_ref)}. Shape: {psi_ref.shape}, dtype: {psi_ref.dtype}")
        if len(kappa_ref) != horizon_steps:
            raise ValueError(f"kappa_ref length mismatch: expected {horizon_steps}, got {len(kappa_ref)}. Shape: {kappa_ref.shape}, dtype: {kappa_ref.dtype}")
        if len(v_ref) != horizon_steps:
            raise ValueError(f"v_ref length mismatch: expected {horizon_steps}, got {len(v_ref)}. Shape: {v_ref.shape}, dtype: {v_ref.dtype}")
        if len(grade_ref) != horizon_steps:
            raise ValueError(f"grade_ref length mismatch: expected {horizon_steps}, got {len(grade_ref)}. Shape: {grade_ref.shape}, dtype: {grade_ref.dtype}")
        
        return (psi_ref, kappa_ref, v_ref, grade_ref, nearest_idx, float(s_0), s_horizon)

