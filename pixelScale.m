function s = pixelScale()
% Pixel measurements are scaled by this number to make the noise statistics
% of lidar and camera match. This is because JPDA tracker allows specifying
% the a single assignment threshold and clutter density.

% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

s = 10;
end