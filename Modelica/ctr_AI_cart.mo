block Controller
  import Modelica.Constants.pi;
  import C.*;
  import K;

  parameter Integer id = 1;
  parameter Boolean is_AI = (id == 1) annotation(Evaluate=false);

  parameter Real Kvx = 35;
  parameter Real Kvy = 40;
  parameter Real Kvz = 100;

  parameter Real Kpx = 1.5;
  parameter Real Kpy = 1.5;
  parameter Real Kpz = 1.5;

  parameter Real epsilon = 1e-2;
  parameter Real maxThrust = 3 * (K.g * K.m);
  parameter Real theta_max = 0.78;

  parameter Real theta_weights[595] = fill(0.0, 595) annotation(Evaluate=false);

  C.InputReal setvr;
  C.InputReal setvtheta;
  C.InputReal setvphi;
  C.InputReal p_target_global[3];

  C.InputReal x, y, z;
  C.InputReal Vx, Vy, Vz;
  C.InputReal grid[K.grid_size];

  // Uscite
  C.OutputReal Trustx, Trusty, Trustz;
  C.OutputReal Vr, Vtheta, Vphi;
  C.OutputReal ctrset_r, ctrset_theta, ctrset_phi;
  
  C.OutputReal heading_out;

  discrete Real grid_vals[K.grid_size];
  Real setvx, setvy, setvz;
  Real rawTrustx, rawTrusty, rawTrustz;
  Real current_horizontal, max_horiz_allowed, tilt_scale;
  Real limitedTrustx, limitedTrusty, limitedTrustz;
  Real total_mag, scale;

  Real target_p_rel_local[3];
  Real Vx_local, Vy_local, Vz_local;

  Real safe_vx_l;
  Real safe_vy_l;
  Real safe_vz_l;

  // Stati discreti AI
  discrete Real safe_vx_local_ai(start = 0.0);
  discrete Real safe_vy_local_ai(start = 0.0);
  discrete Real safe_vz_local_ai(start = 0.0);
  discrete Real heading_mem(start = 0.0);

initial equation
  safe_vx_local_ai = 0.0;
  safe_vy_local_ai = 0.0;
  safe_vz_local_ai = 0.0;
  heading_mem      = 0.0;

equation
  heading_out = heading_mem;

  target_p_rel_local[1] =  (p_target_global[1] - x) * cos(heading_mem) + (p_target_global[2] - y) * sin(heading_mem);
  target_p_rel_local[2] = -(p_target_global[1] - x) * sin(heading_mem) + (p_target_global[2] - y) * cos(heading_mem);
  target_p_rel_local[3] =  (p_target_global[3] - z);

  Vx_local =  Vx * cos(heading_mem) + Vy * sin(heading_mem);
  Vy_local = -Vx * sin(heading_mem) + Vy * cos(heading_mem);
  Vz_local =  Vz;

  if is_AI then
    safe_vx_l = safe_vx_local_ai;
    safe_vy_l = safe_vy_local_ai;
    safe_vz_l = safe_vz_local_ai;

    setvx = safe_vx_l * cos(heading_mem) - safe_vy_l * sin(heading_mem);
    setvy = safe_vx_l * sin(heading_mem) + safe_vy_l * cos(heading_mem);
    setvz = safe_vz_l;

  elseif id == 1 then
    safe_vx_l = 0.0;
    safe_vy_l = 0.0;
    safe_vz_l = 0.0;

    setvx = Kpx * (p_target_global[1] - x);
    setvy = Kpy * (p_target_global[2] - y);
    setvz = Kpz * (p_target_global[3] - z);

  else
    safe_vx_l = 0.0;
    safe_vy_l = 0.0;
    safe_vz_l = 0.0;

    setvx = setvr * sin(setvtheta) * cos(setvphi);
    setvy = setvr * sin(setvtheta) * sin(setvphi);
    setvz = setvr * cos(setvtheta);
  end if;

  rawTrustx = Kvx * (setvx - Vx);
  rawTrusty = Kvy * (setvy - Vy);
  rawTrustz = Kvz * (setvz - Vz) + (K.m * K.g);

  current_horizontal = sqrt(rawTrustx^2 + rawTrusty^2 + 1e-6);
  max_horiz_allowed  = max(0, rawTrustz) * tan(theta_max);

  tilt_scale = if current_horizontal > max_horiz_allowed 
               then max_horiz_allowed / current_horizontal 
               else 1.0;

  limitedTrustx = rawTrustx * tilt_scale;
  limitedTrusty = rawTrusty * tilt_scale;
  limitedTrustz = max(0, rawTrustz);

  total_mag = sqrt(limitedTrustx^2 + limitedTrusty^2 + limitedTrustz^2 + 1e-6);
  scale     = if total_mag > maxThrust then maxThrust / total_mag else 1.0;

  Trustx = limitedTrustx * scale;
  Trusty = limitedTrusty * scale;
  Trustz = limitedTrustz * scale;

  // Telemetria
  Vr     = sqrt(Vx^2 + Vy^2 + Vz^2);
  Vtheta = acos(Vz / (Vr + epsilon));
  Vphi   = Modelica.Math.atan2(sin(atan2(Vy, Vx) - heading_mem), cos(atan2(Vy, Vx) - heading_mem));

  ctrset_r     = sqrt(setvx^2 + setvy^2 + setvz^2);
  ctrset_theta = acos(setvz / (ctrset_r + epsilon));
  ctrset_phi   = atan2(setvy, setvx);

algorithm
  when sample(0, K.T) then
    for i in 1:K.grid_size loop
      grid_vals[i] := grid[i];
    end for;
    
    if sqrt(Vx^2 + Vy^2) > 0.05 then
      heading_mem := atan2(Vy, Vx);
    end if;

    if is_AI then
      (safe_vx_local_ai, safe_vy_local_ai, safe_vz_local_ai) := callFFNN(
        grid_vals,
        target_p_rel_local,
        Vx_local,
        Vy_local,
        Vz_local,
        theta_weights
      );
    end if;
  end when;

end Controller;
