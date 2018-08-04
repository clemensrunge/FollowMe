function [output] = center_arround_min_max(input)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
in_max = max(input);
in_min = min(input);
shift = (in_max + in_min)/2;
output = input - shift;
end

