"""Reference trajectory builder for MPC.

Builds reference heading and curvature profiles from waypoint lists
for MPC horizon prediction.
"""

import numpy as np
from typing import List, Tuple, Optional
import math


class ReferenceBuilder:
    """Builds reference trajectories from waypoints for MPC.
    
    Takes a list of waypoints and builds reference heading (psi_ref)
    and curvature (kappa_ref) arrays for the MPC prediction horizon.
    """
    
    def __init__(self, resample_dist: float = 0.2, curvature_smoothing_num: int = 15):
        """Initialize reference builder.
        
        Args:
            resample_dist: Distance between resampled waypoints (meters)
            curvature_smoothing_num: Number of points apart for curvature calculation (smoothing)
        """
        self.resample_dist = resample_dist
        self.curvature_smoothing_num = curvature_smoothing_num
        self._last_nearest_idx = 0
    
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
        
        Args:
            position: Current vehicle position (x, y)
            waypoints: List of waypoint (x, y) tuples
            last_idx: Last known waypoint index (for forward-only search)
            adaptive_search: If True, adapt search window based on CTE
            cte_magnitude: Current CTE magnitude (meters) for adaptive search
            
        Returns:
            Index of nearest waypoint
        """
        if not waypoints or len(waypoints) == 0:
            return 0
        
        px, py = position
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
            wx, wy = waypoints[i]
            dx = px - wx
            dy = py - wy
            dist2 = dx*dx + dy*dy
            
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_idx = i
        
        self._last_nearest_idx = best_idx
        return best_idx
    
    def resample_waypoints(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Resample waypoints to uniform spacing.
        
        Args:
            waypoints: Original waypoint list
            
        Returns:
            Resampled waypoint list with uniform spacing
        """
        if len(waypoints) < 2:
            return waypoints
        
        resampled = [waypoints[0]]
        current_dist = 0.0
        
        for i in range(1, len(waypoints)):
            x0, y0 = waypoints[i-1]
            x1, y1 = waypoints[i]
            dx = x1 - x0
            dy = y1 - y0
            seg_len = math.sqrt(dx*dx + dy*dy)
            
            if seg_len < 1e-6:
                continue
            
            # Add points along this segment
            while current_dist + self.resample_dist < seg_len:
                current_dist += self.resample_dist
                u = current_dist / seg_len
                resampled.append((x0 + u*dx, y0 + u*dy))
            
            # Move to next segment
            current_dist = current_dist + self.resample_dist - seg_len
            if current_dist < 0:
                current_dist = 0.0
        
        # Always include last waypoint
        if resampled[-1] != waypoints[-1]:
            resampled.append(waypoints[-1])
        
        return resampled
    
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
                       cte_magnitude: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        """Build reference trajectory for MPC horizon.
        
        Args:
            waypoints: List of waypoint (x, y) tuples
            current_position: Current vehicle position (x, y)
            current_heading: Current vehicle heading (radians)
            horizon_steps: Number of prediction steps
            dt: Time step (seconds)
            speed: Current vehicle speed (m/s)
            last_waypoint_idx: Last known waypoint index
            
        Returns:
            Tuple of (psi_ref, kappa_ref, v_ref, new_waypoint_idx)
            - psi_ref: Reference heading array (radians)
            - kappa_ref: Reference curvature array (1/meters)
            - v_ref: Reference speed array (m/s) - currently constant
            - new_waypoint_idx: Updated waypoint index
        """
        if not waypoints or len(waypoints) < 2:
            # Return zero references if no waypoints
            return (
                np.zeros(horizon_steps),
                np.zeros(horizon_steps),
                np.full(horizon_steps, speed),
                0
            )
        
        # Find nearest waypoint
        # Use provided CTE magnitude if available, otherwise estimate from distance to last waypoint
        cte_estimate = cte_magnitude
        if cte_estimate is None and last_waypoint_idx is not None and last_waypoint_idx < len(waypoints):
            last_wp = waypoints[last_waypoint_idx]
            dx = current_position[0] - last_wp[0]
            dy = current_position[1] - last_wp[1]
            cte_estimate = (dx*dx + dy*dy) ** 0.5
        
        nearest_idx = self.find_nearest_waypoint(
            current_position, waypoints, last_waypoint_idx,
            adaptive_search=True, cte_magnitude=cte_estimate
        )
        
        # Ensure horizon_steps is a positive integer
        horizon_steps = int(horizon_steps)
        if horizon_steps <= 0:
            raise ValueError(f"horizon_steps must be > 0, got {horizon_steps}")
        
        # Build reference arrays - ensure they are 1D numpy arrays
        psi_ref = np.zeros(horizon_steps, dtype=np.float64)
        kappa_ref = np.zeros(horizon_steps, dtype=np.float64)
        v_ref = np.full(horizon_steps, float(speed), dtype=np.float64)  # Constant speed for now
        
        # Distance to travel over horizon
        horizon_dist = speed * horizon_steps * dt
        
        # Build reference by walking along waypoints
        current_idx = nearest_idx
        accumulated_dist = 0.0
        
        # Helper function to flip heading by 180° if opposite to vehicle heading
        def adjust_heading_if_opposite(seg_heading, vehicle_heading):
            """Flip segment heading by 180° if it's opposite to vehicle heading (>90° difference)."""
            import math
            heading_diff = seg_heading - vehicle_heading
            # Normalize to [-pi, pi]
            heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
            if abs(heading_diff) > math.pi / 2:  # > 90 degrees
                # Flip by 180°
                flipped = seg_heading + math.pi
                return math.atan2(math.sin(flipped), math.cos(flipped))  # Normalize to [-pi, pi]
            return seg_heading
        
        for k in range(horizon_steps):
            target_dist = speed * (k + 1) * dt
            
            # Find waypoint segment for this step
            while current_idx < len(waypoints) - 1:
                x0, y0 = waypoints[current_idx]
                x1, y1 = waypoints[current_idx + 1]
                dx = x1 - x0
                dy = y1 - y0
                seg_len = math.sqrt(dx*dx + dy*dy)
                
                if seg_len < 1e-6:
                    current_idx += 1
                    continue
                
                if accumulated_dist + seg_len >= target_dist:
                    # Interpolate within this segment
                    u = (target_dist - accumulated_dist) / seg_len
                    ref_x = x0 + u * dx
                    ref_y = y0 + u * dy
                    
                    # Compute heading (tangent direction)
                    seg_heading = math.atan2(dy, dx)
                    # Flip by 180° if opposite to vehicle heading
                    psi_ref[k] = adjust_heading_if_opposite(seg_heading, current_heading)
                    
                    # Compute curvature (use 3-point method with smoothing if possible)
                    # Use points that are curvature_smoothing_num apart for smoother curvature
                    smoothing_offset = max(1, self.curvature_smoothing_num)
                    if current_idx >= smoothing_offset and current_idx < len(waypoints) - smoothing_offset:
                        kappa_ref[k] = self.compute_curvature(
                            waypoints[current_idx - smoothing_offset],
                            waypoints[current_idx],
                            waypoints[current_idx + smoothing_offset]
                        )
                    elif current_idx > 0 and current_idx < len(waypoints) - 1:
                        # Fallback to adjacent points if smoothing not possible
                        kappa_ref[k] = self.compute_curvature(
                            waypoints[current_idx - 1],
                            waypoints[current_idx],
                            waypoints[current_idx + 1]
                        )
                    
                    break
                else:
                    accumulated_dist += seg_len
                    current_idx += 1
            
            # If we've reached the end, use last waypoint
            if current_idx >= len(waypoints) - 1:
                if len(waypoints) >= 2:
                    last_seg = waypoints[-1]
                    prev_seg = waypoints[-2]
                    dx = last_seg[0] - prev_seg[0]
                    dy = last_seg[1] - prev_seg[1]
                    seg_heading = math.atan2(dy, dx)
                    # Flip by 180° if opposite to vehicle heading
                    psi_ref[k] = adjust_heading_if_opposite(seg_heading, current_heading)
                else:
                    psi_ref[k] = current_heading
                    kappa_ref[k] = 0.0
        
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
        
        # Ensure arrays are 1D with correct length
        if psi_ref.ndim != 1:
            raise ValueError(f"psi_ref must be 1D, got {psi_ref.ndim}D array with shape {psi_ref.shape}")
        if kappa_ref.ndim != 1:
            raise ValueError(f"kappa_ref must be 1D, got {kappa_ref.ndim}D array with shape {kappa_ref.shape}")
        if v_ref.ndim != 1:
            raise ValueError(f"v_ref must be 1D, got {v_ref.ndim}D array with shape {v_ref.shape}")
        
        # Verify arrays have correct length - this is the critical check
        if len(psi_ref) != horizon_steps:
            raise ValueError(f"psi_ref length mismatch: expected {horizon_steps}, got {len(psi_ref)}. Shape: {psi_ref.shape}, dtype: {psi_ref.dtype}")
        if len(kappa_ref) != horizon_steps:
            raise ValueError(f"kappa_ref length mismatch: expected {horizon_steps}, got {len(kappa_ref)}. Shape: {kappa_ref.shape}, dtype: {kappa_ref.dtype}")
        if len(v_ref) != horizon_steps:
            raise ValueError(f"v_ref length mismatch: expected {horizon_steps}, got {len(v_ref)}. Shape: {v_ref.shape}, dtype: {v_ref.dtype}")
        
        return (psi_ref, kappa_ref, v_ref, nearest_idx)

