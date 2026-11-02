function [estimate, lower, upper] = proportionCI95(values)
%PROPORTIONCI95 Estimate a binary event rate and its Wilson 95% interval.
% Inputs: finite binary numeric/logical outcomes, one value per run.
% Outputs: event proportion and bounded confidence limits in [0,1].
% Example: [rate,lo,hi] = proportionCI95([false;true;false]);

if ~isnumeric(values) && ~islogical(values)
    error('proportionCI95:InvalidValues', ...
        'values must be a numeric or logical vector of binary outcomes.');
end
if isempty(values)
    estimate = nan;
    lower = nan;
    upper = nan;
    return;
end
if ~isvector(values) || any(~isfinite(double(values(:)))) ...
        || any(values(:) ~= 0 & values(:) ~= 1)
    error('proportionCI95:InvalidValues', ...
        'values must contain only finite zeros and ones.');
end
count = numel(values);
estimate = nnz(values(:))/count;
z = 1.96;
denominator = 1+z^2/count;
center = (estimate+z^2/(2*count))/denominator;
halfWidth = z*sqrt(estimate*(1-estimate)/count ...
    +z^2/(4*count^2))/denominator;
lower = max(0,center-halfWidth);
upper = min(1,center+halfWidth);
end
