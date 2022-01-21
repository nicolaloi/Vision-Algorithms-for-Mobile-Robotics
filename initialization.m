function [S_0,S_initial,Twc_initial,Tcw_initial] = initialization(img1,img2,camera_intrinsics,params)

%restart initialization until mean reprojection error is below 1
mean_reprojection_error = 10;
init_counter = 0;
S_initial.keypoints = [];
while mean_reprojection_error > 1 || height(S_initial.keypoints) < params.init_min_keypoints
    if init_counter ~= 0
        disp(['initialization failed - retrying.. ',num2str(init_counter)])
    end

    %detect keypoints
    keypoints1 = params.feature_detector_init(img1);
    keypoints2 = params.feature_detector_init(img2);

    %create descriptors
    [descriptors1,valid_points1] = params.extract_feature_init(img1,keypoints1);
    [descriptors2,valid_points2] = params.extract_feature_init(img2,keypoints2);

    %match keypoints
    matched_indices = matchFeatures(descriptors1,descriptors2);
    matched_points1 = valid_points1(matched_indices(:,1),:);
    matched_points2 = valid_points2(matched_indices(:,2),:);

    %estimate fundamental matrix
    [F,inlier_indices] = params.estimate_fundamental_matrix(matched_points1,matched_points2);
    inlier_points1 = matched_points1(inlier_indices,:);
    inlier_points2 = matched_points2(inlier_indices,:);

    %determine relative camera pose
    [relative_orientation,relative_location] = relativeCameraPose(F,camera_intrinsics,inlier_points1,inlier_points2); %R_wc, t_wc
    if numel(relative_location) == 3 %avoid invalid estimations where relativeCameraPose doesn't yield a unique translation vector (a rarely occuring issue for parking)
        [rotation_matrix,translation_vector] = cameraPoseToExtrinsics(relative_orientation,relative_location); %R_cw, t_cw
        Twc_initial = rigid3d(relative_orientation,relative_location);
        Tcw_initial = rigid3d(rotation_matrix,translation_vector);
        
        %triangulate keypoints and determine reprojection error
        camera_extrinsics1 = rigid3d;
        camera_extrinsics2 = Tcw_initial;
        cam_matrix1 = cameraMatrix(camera_intrinsics,camera_extrinsics1);
        cam_matrix2 = cameraMatrix(camera_intrinsics,camera_extrinsics2);
        [triangulated_points,reprojection_errors,validity] = triangulate(inlier_points1,inlier_points2,cam_matrix1,cam_matrix2);
        S_0.keypoints = inlier_points1.Location(validity,:);
        S_0.landmarks = triangulated_points(validity,:);
        S_initial.keypoints = inlier_points2.Location(validity,:);
        S_initial.landmarks = triangulated_points(validity,:);
        reprojection_errors = reprojection_errors(validity,:);
        mean_reprojection_error = mean(reprojection_errors);
    else
        mean_reprojection_error = 10;
    end

    init_counter = init_counter+1;
end
disp(['initialization successful!'])

if params.BA == 1
    %Perform bundle adjustment for the 2 initilization frames
    ViewId = uint32([0;1]);
    pointTracks(1,height(S_initial.keypoints)) = pointTrack();
    for i = 1:height(S_initial.keypoints)
        pointTracks(i) = pointTrack(ViewId,[S_0.keypoints(i,:);S_initial.keypoints(i,:)]);
    end
    Orientation = {eye(3);Twc_initial.Rotation};
    Location = {zeros(1,3);Twc_initial.Translation};
    cameraPoses = table(ViewId,Orientation,Location);
    disp(['-----------------------------------------------------------'])
    [S_0.landmarks,cameraPoses] = bundleAdjustment(S_0.landmarks,pointTracks,cameraPoses,camera_intrinsics,'AbsoluteTolerance',params.BA_absolute_tolerance,'FixedViewIDs',0,'PointsUndistorted',true,'Verbose',true);
    disp(['-----------------------------------------------------------'])
    S_initial.landmarks = S_initial.landmarks;
    Twc_initial = rigid3d(cell2mat(cameraPoses.Orientation(2)),cell2mat(cameraPoses.Location(2)));
    Tcw_initial = rigid3d(Twc_initial.Rotation',-Twc_initial.Translation*Twc_initial.Rotation');
end

%for plotting
global num_landmarks
global full_landmarks
global full_trajectory
num_landmarks(1) = height(S_initial.landmarks);
num_landmarks(2) = height(S_initial.landmarks);
if params.ds ~=2
    full_landmarks{1} = S_initial.landmarks;
    full_landmarks{2} = S_initial.landmarks;
elseif  params.ds ==2 
    %initial landmarks of parking often lie extremely far away due to the
    %nature of the dataset, skewing the plot scale
    full_landmarks{1} = single(zeros(height(S_initial.landmarks),3));
    full_landmarks{2} = single(zeros(height(S_initial.landmarks),3));
end
full_trajectory = [zeros(1,3);Twc_initial.Translation];
end