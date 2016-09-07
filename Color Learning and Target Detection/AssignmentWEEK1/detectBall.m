% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
load('mu.mat', 'mu');
load('sig.mat', 'sig');
thre = 8e-6;

% Find ball-color pixels using your model
dI = double(I);
row = size(dI, 1); 
col = size(dI, 2);
for i = 1:3
    dI(:,:,i) = dI(:,:,i) - mu(i);
end
dI = reshape(dI, row*col, 3);
dI = exp(-0.5 * sum(dI * inv(sig) .* dI, 2)) ./ (2 * pi)^1.5 ./ det(sig)^0.5;
dI = reshape(dI, row, col);

% Do more processing to segment out the right cluster of pixels.
dI = dI > thre;
segI = false(size(dI));
CC = bwconncomp(dI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[~,idx] = max(numPixels);
segI(CC.PixelIdxList{idx}) = true; 
% figure, imshow(segI); hold on;

% Compute the location of the ball center
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
% plot(loc(1), loc(2),'r+');

% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
