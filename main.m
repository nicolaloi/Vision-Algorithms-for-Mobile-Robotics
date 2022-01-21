clear all
close all
clc

addpath("datasets\")

%% Select Dataset:
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

%% Set the Parameters:
%% 0: KITTI
% initialization:
params0.ds = 0; %dataset label
params0.init_frames = [0 4]; %frame pair used for initilization/bootstrapping
params0.feature_detector_init = @(x)detectMinEigenFeatures(x,'MinQuality',0.01,'FilterSize',7); %feature detector initialization
params0.extract_feature_init = @(x,y)extractFeatures(x,y,'BlockSize',11); %feature extractor initialization
params0.estimate_fundamental_matrix = @(x,y)estimateFundamentalMatrix(x,y,'Method','RANSAC','NumTrials',50000,'DistanceThreshold',1,'Confidence',99.99); %estimate fundamental matrix function initialization
params0.init_min_keypoints = 0; %minimum number of keypoints required after initialization (set to 0 to disable) 

% processFrame:
params0.alpha_threshold = 0.25/180*pi; % threshold angle to add candidate
params0.reprojection_error_threshold = 1; % reprojection error threshold to add candidate
params0.min_keypoints_threshold = 80; %increase reprojection error threshold for candidates for one iteration if number of tracked keypoints drops below this value (0 deactivates this feature)
params0.feature_detector = @(x,y,z)detectMinEigenFeatures(x,'MinQuality',y,'FilterSize',z); %feature detector
params0.feature_quality = 0.01; %quality of the detected Shi-Tomasi features
params0.filter_size = 7; %filter size of the Shi-Tomasi feature detector
params0.candidates_object = @cornerPoints; %feature points object
params0.estimate_world_camera_pose = @(x,y,z)estimateWorldCameraPose(x,y,z,'MaxReprojectionError',1,'Confidence',99.99,'MaxNumTrials',25000); %estimate world camera pose function
params0.pixel_distance_threshold = 2; %distance threshold for detecting new features in pixels
params0.select_uniform_threshold = 150; %threshold for the maximum number of new candidates to track - candidates are sampled uniformly if more new candidates than this are found (set to very large value to disable)
params0.KLT = [4,1,5,50]; %arguments of the KLT track - [# pyramid levels, forward-backward error threshold, block size, maximum iterations]
params0.BA_n_views = 10; %size of the BA sliding window for KITTI
params0.BA_absolute_tolerance = 0.01; %target reprojection error of BA (set to very low values to use gradient as termination criterion for the optimization)

%plotting
params0.n_current_landmarks_plotted = 150; %maximum number of current landmarks to plot in local trajectory
params0.percentile_all_landmarks = 75; %upper percentile boundary for current landmarks to plot in full trajectory
params0.past_poses_plotted = 20; %past poses to plot in local trajectory

%% 1: Malaga:
% initialization:
params1.ds = 1; %dataset label
params1.init_frames = [1 4]; %frame pair used for initilization/bootstrapping
params1.feature_detector_init = @(x)detectMinEigenFeatures(x,'MinQuality',0.06,'FilterSize',13); %feature detector initialization
params1.extract_feature_init = @(x,y)extractFeatures(x,y); %feature extractor initialization
params1.estimate_fundamental_matrix = @(x,y)estimateFundamentalMatrix(x,y,'Method','RANSAC','NumTrials',50000,'DistanceThreshold',1,'Confidence',99.99); %estimate fundamental matrix function initialization
params1.init_min_keypoints = 0; %minimum number of keypoints required after initialization (set to 0 to disable) 

% processFrame:
params1.alpha_threshold = 0.25/180*pi; %threshold angle to add candidate (has to be smaller than pi/2)
params1.reprojection_error_threshold = 1; %reprojection error threshold to add candidate
params1.min_keypoints_threshold = 100; %increase reprojection error threshold for candidates for one iteration if number of tracked keypoints drops below this value (0 deactivates this feature)
params1.feature_detector = @(x,y,z)detectMinEigenFeatures(x,'MinQuality',y,'FilterSize',z); %feature detector
params1.feature_quality = 0.03; %quality of the detected Shi-Tomasi features
params1.filter_size = 7; %filter size of the Shi-Tomasi feature detector
params1.candidates_object = @cornerPoints; %feature points object
params1.estimate_world_camera_pose = @(x,y,z)estimateWorldCameraPose(x,y,z,'MaxReprojectionError',1,'Confidence',99.99,'MaxNumTrials',15000); %estimate world camera pose function
params1.pixel_distance_threshold = 2; %distance threshold for detecting new features in pixels
params1.select_uniform_threshold = 250; %threshold for the maximum number of new candidates to track - candidates are sampled uniformly if more new candidates than this are found (set to very large value to disable)
params1.KLT = [4,1,5,50]; %arguments of the KLT track - [# pyramid levels, forward-backward error threshold, block size, maximum iterations]
params1.BA_n_views = 10; %size of the BA sliding window for Malaga
params1.BA_absolute_tolerance = 0.01; %target reprojection error of BA (set to very low values to use gradient as termination criterion for the optimization)

%plotting
params1.n_current_landmarks_plotted = 100; %maximum number of current landmarks to plot in local trajectory
params1.percentile_all_landmarks = 75; %upper percentile boundary for current landmarks to plot in full trajectory
params1.past_poses_plotted = 20; %past poses to plot in local trajectory

%% 2: parking:
% initialization:
params2.ds = 2; %dataset label
params2.init_frames = [0 3]; %frame pair used for initilization/bootstrapping
params2.feature_detector_init = @(x)detectMinEigenFeatures(x,'MinQuality',0.0001,'FilterSize',11); %feature detector initialization
params2.extract_feature_init = @(x,y)extractFeatures(x,y); %feature extractor initialization
params2.estimate_fundamental_matrix = @(x,y)estimateFundamentalMatrix(x,y,'Method','RANSAC','NumTrials',50000,'DistanceThreshold',1,'Confidence',99.99); %estimate fundamental matrix function initialization
params2.init_min_keypoints = 100; %minimum number of keypoints required after initialization (set to 0 to disable) 

% processFrame:
params2.alpha_threshold = 2.5/180*pi; % threshold angle to add candidate
params2.reprojection_error_threshold = 1; % reprojection error threshold to add candidate
params2.min_keypoints_threshold = 0; %increase reprojection error threshold for candidates for one iteration if number of tracked keypoints drops below this value (0 deactivates this feature)
params2.feature_detector = @(x,y,z)detectMinEigenFeatures(x,'MinQuality',y,'FilterSize',z); %feature detector
params2.feature_quality = 0.001; %quality of the detected Shi-Tomasi features
params2.filter_size = 5; %filter size of the Shi-Tomasi feature detector
params2.candidates_object = @cornerPoints; %feature points object
params2.estimate_world_camera_pose = @(x,y,z)estimateWorldCameraPose(x,y,z,'MaxReprojectionError',1,'Confidence',99.99,'MaxNumTrials',15000); %estimate world camera pose function
params2.pixel_distance_threshold = 2; %distance threshold for detecting new features in pixels
params2.select_uniform_threshold = 200; %threshold for the maximum number of new candidates to track - candidates are sampled uniformly if more new candidates than this are found (set to very large value to disable)
params2.KLT = [4,1,7,50]; %arguments of the KLT track - [# pyramid levels, forward-backward error threshold, block size, maximum iterations]
params2.BA_n_views = 15; %size of the BA sliding window for Parking
params2.BA_absolute_tolerance = 0.1; %target reprojection error of BA (set to very low values to use gradient as termination criterion for the optimization)

%plotting
params2.n_current_landmarks_plotted = 300; %maximum number of current landmarks to plot in local trajectory
params2.percentile_all_landmarks = 85; %upper percentile boundary for current landmarks to plot in full trajectory
params2.past_poses_plotted = 20; %past poses to plot in local trajectory

%% select parameter struct corresponding to dataset
params_all_sets = {params0, params1, params2};
params = params_all_sets{ds+1};

%% include n-view Bundle Adjustment (bonus feature)
params.BA = 0; %1: activate / 0: deactivate

%% Setup
if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'datasets/kitti';
    %assert(exist(kitti_path, 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end-4 end]);
    last_frame = 2760;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = 'datasets/malaga-urban-dataset-extract-07';
    %assert(exist(malaga_path, 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = 'datasets/parking';
    %assert(exist(parking_path, 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end-4 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    bootstrap_frames = params.init_frames;
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img2 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    bootstrap_frames = params.init_frames;
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img2 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    bootstrap_frames = params.init_frames;
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));  
else
    assert(false);
end

camera_intrinsics = cameraIntrinsics([K(1,1) K(2,2)],[K(1,3) K(2,3)],[size(img1,1),size(img1,2)]);
[S_0,S_initial,Twc_initial,Tcw_initial] = initialization(img1,img2,camera_intrinsics,params);

%% Continuous operation
if params.BA == 1
    %create table to store data of past n frames for bundle adjustment
    %initialize table using data from initilization frames
    S_initial.indices = (1:height(S_initial.keypoints))';
    ViewId = uint32([0;params.init_frames(2)]);
    Orientation = {eye(3,3);Twc_initial.Rotation};
    Location = {zeros(1,3);Twc_initial.Translation};
    Keypoints = {S_0.keypoints;S_initial.keypoints};
    Landmarks = {S_0.landmarks;S_initial.landmarks};
    Indices = {S_initial.indices;S_initial.indices};
    global largest_index
    largest_index = uint32([S_initial.indices(end);S_initial.indices(end)]);
    BA = table(ViewId,Orientation,Location,Keypoints,Landmarks,Indices,largest_index);
    largest_index = BA.largest_index(1);
else
    BA = [];
end
%initialize previous state struct
%assign state of second initilization frame
prev_S = S_initial;
prev_S.candidate_keypoints = [];
prev_S.first_observation = [];
prev_S.candidate_poses = [];
prev_img = img2;
% initialize KLT
global KLT_tracker 
KLT_tracker = vision.PointTracker('NumPyramidLevels',params.KLT(1),'MaxBidirectionalError',params.KLT(2),'BlockSize',[2*params.KLT(3)+1, 2*params.KLT(3)+1],'MaxIterations',params.KLT(4));
% for plotting
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

% VO process frame
for i = (bootstrap_frames(2)+1):last_frame
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end

    [S,BA] = processFrame(image,prev_img,prev_S,camera_intrinsics,BA,i,params,last_frame);
    prev_img = image;
    prev_S = S;

    % Makes sure that plots refresh.    
    pause(0.01);
end