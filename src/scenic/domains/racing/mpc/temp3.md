### 3) Pre-cache fellow hot paths too (MAPort first-touch latency reduction)

You already precache ego + control paths in `simulator.py` (nice).
If your test includes fellows, add fellow array paths as hot refs too.

### Add these to `hot_paths` in `simulator.py` setup

* Fellow readback arrays:

  * `.../FellowTrailer/x`
  * `.../FellowTrailer/y`
  * `.../FellowTrailer/z`
  * `.../FellowTrailer/yaw_deg_out`
  * `.../FellowTrailer/v_Fellows`
  * `.../FellowTrailer/w_Fellows`
* Fellow external write arrays:

  * `.../External_Signals/Const_v_Fellows_External[km|h]/Value`
  * `.../External_Signals/Const_d_Fellows_External[m]/Value`
