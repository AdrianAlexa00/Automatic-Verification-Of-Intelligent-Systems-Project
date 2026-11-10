block Setpoint
  import Modelica.Constants.pi;
  import K;
  import C.*;

  C.OutputReal target_p_hero[3];
  C.OutputReal  setvr[K.N];
  C.OutputReal  setvtheta[K.N];
  C.OutputReal  setvphi[K.N];

  parameter Real hero_target_x = 0.0 annotation(Evaluate=false);
  parameter Real hero_target_y = 0.0 annotation(Evaluate=false);
  parameter Real hero_target_z = 0.0 annotation(Evaluate=false);

  parameter Real target_vphi[K.N] = fill(0.0, K.N) annotation(Evaluate=false);
  parameter Real target_vr[K.N] = fill(2.0, K.N) annotation(Evaluate=false);
  parameter Real target_vtheta[K.N] = fill(pi/2, K.N) annotation(Evaluate=false);

equation  
  target_p_hero[1] = hero_target_x;
  target_p_hero[2] = hero_target_y;
  target_p_hero[3] = hero_target_z;

  for i in 1:K.N loop
    if i == 1 then
      setvr[i] = target_vr[i];
      setvtheta[i] = target_vtheta[i];
      setvphi[i] = target_vphi[i];
    else
      setvr[i] = max(0.5, target_vr[i] + 0.8 * sin(0.5 * time + i * 1.2));
      setvtheta[i] = target_vtheta[i] + 0.35 * cos(0.6 * time + i * 2.1);
      setvphi[i] = target_vphi[i] + 0.6 * sin(0.8 * time + i * 1.5);
    end if;
  end for;
end Setpoint;
