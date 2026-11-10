model MultiUAVSystem
  import C.*;
  import K;

  UAV drones[K.N](id = 1:K.N);
  Setpoint p;
  Real swarmBus[K.N, 3];

  inner Modelica.Blocks.Noise.GlobalSeed globalSeed(fixedSeed=42);
  
  parameter Boolean stop_on_reach = false "IF true, stop the simulation before the maximum time" annotation(Evaluate=false);
  
  
  Real hero_dist_to_target;
  
equation
  for i in 1:K.N loop
    swarmBus[i, 1] = drones[i].x;
    swarmBus[i, 2] = drones[i].y;
    swarmBus[i, 3] = drones[i].z;
  end for;

  for i in 1:K.N loop
    drones[i].swarmBus = swarmBus;
    drones[i].p_target_global = p.target_p_hero;
    drones[i].setvr = p.setvr[i];
    drones[i].setvtheta = p.setvtheta[i];
    drones[i].setvphi = p.setvphi[i];
  end for;
  hero_dist_to_target = sqrt((drones[1].x - p.target_p_hero[1])^2 + 
                             (drones[1].y - p.target_p_hero[2])^2 + 
                             (drones[1].z - p.target_p_hero[3])^2);
  when stop_on_reach and hero_dist_to_target <= 0.5 then
    terminate("Target B successfully approached: Simulation complete.");
  end when;

end MultiUAVSystem;
