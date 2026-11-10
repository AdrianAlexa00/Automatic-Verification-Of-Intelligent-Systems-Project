
block WindGenerator

  parameter Real meanWind[3] = {0.0, 0.0, 0.0} "Mean wind in X, Y, Z";
  parameter Real noiseAmplitude = 0.0 "Intensity of gusts";
  
  // MANDATORY for 4.x
outer Modelica.Blocks.Noise.GlobalSeed globalSeed;

// Corrected parameters: y_min and y_max
  Modelica.Blocks.Noise.UniformNoise noiseX(y_min=-noiseAmplitude, y_max=noiseAmplitude, samplePeriod=0.01);
  Modelica.Blocks.Noise.UniformNoise noiseY(y_min=-noiseAmplitude, y_max=noiseAmplitude, samplePeriod=0.01);
  Modelica.Blocks.Noise.UniformNoise noiseZ(y_min=-noiseAmplitude, y_max=noiseAmplitude, samplePeriod=0.01);

  Modelica.Blocks.Interfaces.RealOutput windX;
  Modelica.Blocks.Interfaces.RealOutput windY;
  Modelica.Blocks.Interfaces.RealOutput windZ;

equation
  windX = meanWind[1] + noiseX.y;
  windY = meanWind[2] + noiseY.y;
  windZ = meanWind[3] + noiseZ.y;
  
  //annotation(uses(Modelica(version="4.0.0")));
end WindGenerator;
/*block WindGenerator
  parameter Real meanWind[3] = {1.0, 0.5, 1.0} "Mean wind in X, Y, Z";
  parameter Real noiseAmplitude = 0.5 "Intensity of gusts";
  
  // Internal noise components for X, Y, Z
  Modelica.Blocks.Noise.RandomNoise noiseX(amplitude=noiseAmplitude, samplePeriod=0.02);
  Modelica.Blocks.Noise.RandomNoise noiseY(amplitude=noiseAmplitude, samplePeriod=0.02);
  Modelica.Blocks.Noise.RandomNoise noiseZ(amplitude=noiseAmplitude, samplePeriod=0.02);

  Modelica.Blocks.Interfaces.RealOutput windX;
  Modelica.Blocks.Interfaces.RealOutput windY;
  Modelica.Blocks.Interfaces.RealOutput windZ;

equation
  windX = meanWind[1] + noiseX.y;
  windY = meanWind[2] + noiseY.y;
  windZ = meanWind[3] + noiseZ.y;
end WindGenerator;
*/