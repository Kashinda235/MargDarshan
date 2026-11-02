function [value, state] = perceptionRandomUniform(state)
%RANDOMUNIFORM Generate one reproducible uniform variate without global RNG.
% Inputs: state, nonnegative integer PRNG state.
% Outputs: value in (0,1) and updated state.
% Example: [u, state] = perceptionRandomUniform(17);

validateattributes(state, {'numeric'}, ...
    {'scalar','real','finite','integer','nonnegative'}, mfilename, 'state');
modulus = 4294967296;
state = mod(1664525 * double(state) + 1013904223, modulus);
value = (state + 0.5) / modulus;
end
