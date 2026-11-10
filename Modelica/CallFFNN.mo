function callFFNN
  import K;

  input Real grid[K.grid_size];
  input Real target_p_rel_local[3];
  input Real vx_local;
  input Real vy_local;
  input Real vz_local;
  input Real theta_weights[595]; 

  output Real safe_vx_local;
  output Real safe_vy_local;
  output Real safe_vz_local;

  external "C" compute_ffnn_action_v2(
        grid,
        target_p_rel_local,
        vx_local,
        vy_local,
        vz_local,
        theta_weights,
        safe_vx_local,
        safe_vy_local,
        safe_vz_local
  ) annotation(Include="#include \"/path_to_the_FFNN\"");
end callFFNN;
