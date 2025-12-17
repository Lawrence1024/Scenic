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
    
    def __init__(self, resample_dist: float = 0.2):
        """Initialize reference builder.
        
        Args:
            resample_dist: Distance between resampled waypoints (meters)
        """
        self.resample_dist = resample_dist
        self._last_nearest_idx = 0
    
    def find_nearest_waypoint(self, 
                              position: Tuple[float, float],
                              waypoints: List[Tuple[float, float]],
                              last_idx: Optional[int] = None) -> int:
        """Find nearest waypoint index to current position.
        
        Uses forward-only search starting from last_idx to ensure
        forward progress along the path.
        
        Args:
            position: Current vehicle position (x, y)
            waypoints: List of waypoint (x, y) tuples
            last_idx: Last known waypoint index (for forward-only search)
            
        Returns:
            Index of nearest waypoint
        """
        if not waypoints or len(waypoints) == 0:
            return 0
        
        px, py = position
        start_idx = last_idx if last_idx is not None else self._last_nearest_idx
        
        # Search forward from last index (with some lookback for safety)
        search_start = max(0, start_idx - 10)
        search_end = min(len(waypoints), start_idx + 50)
        
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
                       last_waypoint_idx: Optional[int] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
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
        nearest_idx = self.find_nearest_waypoint(
            current_position, waypoints, last_waypoint_idx
        )
        
        # Build reference arrays
        psi_ref = np.zeros(horizon_steps)
        kappa_ref = np.zeros(horizon_steps)
        v_ref = np.full(horizon_steps, speed)  # Constant speed for now
        
        # Distance to travel over horizon
        horizon_dist = speed * horizon_steps * dt
        
        # Build reference by walking along waypoints
        current_idx = nearest_idx
        accumulated_dist = 0.0
        
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
                    psi_ref[k] = math.atan2(dy, dx)
                    
                    # Compute curvature (use 3-point method if possible)
                    if current_idx > 0 and current_idx < len(waypoints) - 1:
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
                    psi_ref[k] = math.atan2(dy, dx)
                else:
                    psi_ref[k] = current_heading
                kappa_ref[k] = 0.0
        
        return (psi_ref, kappa_ref, v_ref, nearest_idx)

