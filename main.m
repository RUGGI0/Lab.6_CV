%% Part 1: estimation of the fundamental matrix with manually selected correspondences

% Load images
img1 = imread('Rubik/Rubik1.pgm');
img2 = imread('Rubik/Rubik2.pgm');

% Load points
P1orig = load('Rubik/Rubik1.points');
P2orig = load('Rubik/Rubik2.points');

n = size(P1orig,1);

% Add the third component to work in homogeneous coordinates
P1 = [P1orig'; ones(1,n)];
P2 = [P2orig'; ones(1,n)];
                    
%8-points algorithm
F = EightPointsAlgorithm(P1, P2);
%8-points algorithm with normalization
Fn = EightPointsAlgorithmN(P1, P2);

%Evaluate the error
Error(F, P1, P2);
Error(Fn, P1, P2);

% Visualize the epipolar lines
visualizeEpipolarLines(img1, img2, F, P1orig, P2orig, 1);
visualizeEpipolarLines(img1, img2, Fn, P1orig, P2orig, 2);

%% Part 2: assessing the use of RANSAC 
clc, clear;

% Load images
img1 = imread('Rubik/Rubik1.pgm');
img2 = imread('Rubik/Rubik2.pgm');

% Load points
P1orig = load('Rubik/Rubik1.points');
P2orig = load('Rubik/Rubik2.points');

% Add random points (to assess RANSAC)
x1r = double(round(size(img1,1)*rand(5,1)));
y1r = double(round(size(img1,2)*rand(5,1)));

x2r = double(round(size(img2,1)*rand(5,1)));
y2r = double(round(size(img2,2)*rand(5,1)));

P1orign = [P1orig; [x1r, y1r]];
P2orign = [P2orig; [x2r, y2r]];

n = size(P1orign,1);

% Add the third component to work in homogeneous coordinates
P1 = [P1orign'; ones(1,n)];
P2 = [P2orign'; ones(1,n)];

% Estimate the fundamental matrix with RANSAC
th = 10^(-2);
%F = EightPointsAlgorithmN(P1, P2);
[F, consensus, outliers] = ransacF(P1, P2, th);

%Evaluate the error
Error(F, P1, P2);

% Visualize the epipolar lines
visualizeEpipolarLines(img1, img2, F, P1orig, P2orig, 3);


%% Part 3: using image matching+ransac
clc, close, clear;
addpath('ImageMatching'); % change the path here if needed

% Load images
img1 = rgb2gray(imread('Images/Img_1.jpg'));
img2 = rgb2gray(imread('Images/Img_2.jpg'));


img1 = imresize(img1, 0.5);
img2 = imresize(img2, 0.5);

% extraction of keypoints and matching
list = imageMatching(img1, img2, 'POSNCC', 0.8, 1, 100);

n = size(list,1);

% Add the third component to work in homogeneous coordinates
P1 = [list(:,2)'; list(:,1)'; ones(1,n)];
P2 = [list(:,4)'; list(:,3)'; ones(1,n)];

% Estimate the fundamental matrix with RANSAC
th = 10^(-2);
[F, consensus, outliers] = ransacF(P1, P2, th);

%Evaluate the error
Error(F, P1, P2);

% Visualize the epipolar lines
visualizeEpipolarLines(img1, img2, F, P1(1:2,:)', P2(1:2,:)', 10);
