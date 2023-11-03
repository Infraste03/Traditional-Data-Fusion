function data = getIntrinsicsData()
% getIntrinsicsData The function returns camera intrinsic data for Pandaset
% cameras.

% This is a helper function and may be removed in a future release.

% Copyright 2022 The MathWorks, Inc.

data = repmat(struct('FocalLength',[0 0],...
        'PrincipalPoint',[0 0],...
        'ImageSize',[0 0],...
        'RadialDistortion',[0 0],...
        'TangentialDistortion',[0 0]),1,6);

focalLengths = 1e3*[1.970013100000000   1.970009100000000
    0.933466700000000   0.934675400000000
    0.929842900000000   0.930059200000000
    0.930040700000000   0.930032400000000
    0.930451400000000   0.930089100000000
    0.922546500000000   0.922422900000000];

principalPoint =   1e2*[9.700002000000000   4.832988000000000
   8.964691999999999   5.073557000000000
   9.721793999999999   5.080057000000000
   9.650525000000000   4.634161000000000
   9.916883000000000   5.416056999999999
   9.450570000000001   5.175750000000001];

imageSize = [1080        1920
             1080        1920
             1080        1920
             1080        1920
             1080        1920
             1080        1920];

for i = 1:numel(data)
    data(i).FocalLength = focalLengths(i,:);
    data(i).PrincipalPoint = principalPoint(i,:);
    data(i).ImageSize = imageSize(i,:);
end

end