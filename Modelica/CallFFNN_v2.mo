function callFFNN
  import K;

  input Real grid[K.grid_size];
  input Real target_p_rel_local[3];
  input Real vr;
  input Real vtheta;
  input Real vphi;
  input Real theta_weights[595]; 

  output Real safe_vr;
  output Real safe_vtheta;
  output Real safe_vphi;

  external "C" compute_ffnn_action_v2(
        grid,
        target_p_rel_local,
        vr,
        vtheta,
        vphi,
        theta_weights,
        safe_vr,
        safe_vtheta,
        safe_vphi
  ) annotation(Include="#include \"path_to_FFNN"");
end callFFNN;
