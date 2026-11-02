function [value, state] = perceptionRandomNormal(state)
%PERCEPTIONRANDOMNORMAL Generate a normal variate without global RNG.
% Inputs: state, nonnegative integer PRNG state.
% Outputs: a standard-normal value and updated PRNG state.
% Example: [z, state] = perceptionRandomNormal(17);

[u1, state] = perceptionRandomUniform(state);
[u2, state] = perceptionRandomUniform(state);
value = sqrt(-2.0 * log(u1)) * cos(2.0 * pi * u2);
end
