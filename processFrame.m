function [S,BA] = processFrame(img,prev_img,prev_S,camera_intrinsics,BA,frame_num,params,last_frame)

tic
%Initialize state struct for current frame
S.keypoints           = [];
S.landmarks           = [];
S.indices             = [];
S.candidate_keypoints = [];
S.first_observation   = [];
S.candidate_poses     = [];
S.candidate_keypoints_invalid_triangulation = [];
S.candidate_keypoints_invalid_reprojection = [];

%determine KLT keypoints in new frame
global KLT_tracker
release(KLT_tracker);
initialize(KLT_tracker,prev_S.keypoints,prev_img);
[S.keypoints,validity_KLT,~] = KLT_tracker(img);
S.keypoints      = S.keypoints(validity_KLT,:);
prev_S.keypoints = prev_S.keypoints(validity_KLT,:);
prev_S.landmarks = prev_S.landmarks(validity_KLT,:);
disp(['fraction of keypoints tracked by KLT: ',num2str(sum(validity_KLT)),' / ',num2str(height(validity_KLT))])

%determine camera pose using P3P and MSAC
[Rwc,twc,inlier_indeces] = params.estimate_world_camera_pose(S.keypoints,prev_S.landmarks,camera_intrinsics);
Twc = rigid3d(Rwc,twc);
Tcw = rigid3d(Twc.Rotation',-Twc.Translation*Twc.Rotation');
S.keypoints = S.keypoints(inlier_indeces,:);
S.landmarks = prev_S.landmarks(inlier_indeces,:);
prev_S.keypoints = prev_S.keypoints(inlier_indeces,:);
prev_S.landmarks = prev_S.landmarks(inlier_indeces,:);
disp(['fraction of keypoints used for world camera pose estimation (MSAC): ',num2str(sum(inlier_indeces)),' / ',num2str(height(inlier_indeces))])

%----------------------------------------------------------------------
%BONUS FEATURE
%perform n-view bundle adjustment on the last n frames
if params.BA == 1 && params.BA_n_views > 0
    [S,prev_S,Twc,Tcw,BA] = nViewBundleAdjustment(S,prev_S,Twc,BA,frame_num,validity_KLT,inlier_indeces,camera_intrinsics,params);
end
%----------------------------------------------------------------------

%dynamically adapt reprojection error threshold based on number of tracked
%landmarks (only for this iteration; reset to original value in next
%iteration)
if height(S.keypoints) < params.min_keypoints_threshold
    params.reprojection_error_threshold = params.reprojection_error_threshold+1;
    if params.ds == 0
        %for KITTI also slightly lower the quality of detected features
        params.feature_quality = 0.5*params.feature_quality;
    end
    disp(['increased reprojection error threshold to: ',num2str(params.reprojection_error_threshold)])
    disp(['decreased feature quality to: ',num2str(params.feature_quality)])
end

%determine which new landmarks to add based on bearing angle and
%reprojection error
if (~isempty(prev_S.candidate_keypoints))
    %first, delete keypoints invalid for too long
    S.candidate_keypoints_invalid_triangulation = prev_S.candidate_keypoints_invalid_triangulation;
    S.candidate_keypoints_invalid_reprojection = prev_S.candidate_keypoints_invalid_reprojection;
    invalid_candidates_triangulation_idx = S.candidate_keypoints_invalid_triangulation > 4; %delete candidates invalid for more than 4 frames in a row
    invalid_candidates_reprojection_idx = S.candidate_keypoints_invalid_reprojection > 4; %delete candidates with high repr. error for more than 4 frames in a row
    invalid_candidates_idx = (invalid_candidates_triangulation_idx | invalid_candidates_reprojection_idx);
    S.candidate_keypoints_invalid_triangulation(invalid_candidates_idx) = [];
    S.candidate_keypoints_invalid_reprojection(invalid_candidates_idx) = [];
    prev_S.candidate_keypoints(invalid_candidates_idx, :) = [];
    prev_S.first_observation(invalid_candidates_idx, :) = [];
    prev_S.candidate_poses(invalid_candidates_idx, :) = [];
    disp(['Invalid triangulated candidate keypoints for too long: ',num2str(sum(invalid_candidates_triangulation_idx))])
    disp(['Reproj. error candidate keypoints for too long: ',num2str(sum(invalid_candidates_reprojection_idx))])

    %update tracked candidate keypoints
    global KLT_tracker
    release(KLT_tracker);
    initialize(KLT_tracker,prev_S.candidate_keypoints,prev_img);
    [klt_candidate_points,candidate_validity,~] = KLT_tracker(img);
    S.candidate_keypoints = klt_candidate_points(candidate_validity,:);
    S.first_observation = prev_S.first_observation(candidate_validity,:);
    S.candidate_poses = prev_S.candidate_poses(candidate_validity,:);
    S.candidate_keypoints_invalid_triangulation = prev_S.candidate_keypoints_invalid_triangulation(candidate_validity);
    S.candidate_keypoints_invalid_reprojection = prev_S.candidate_keypoints_invalid_reprojection(candidate_validity);

    %set parameters
    add_candidates = zeros(height(S.candidate_keypoints),1);
    angle_too_small = zeros(height(S.candidate_keypoints),1);
    reprojection_error_too_large = zeros(height(S.candidate_keypoints),1);
    not_valid = zeros(height(S.candidate_keypoints),1);
    maxangle = 0;

    %check bearing vector angle
    for i=1:size(S.candidate_keypoints,1)
        %triangulate candidate landmark from first and current observation
        %of candidate keypoint
        Twtau = rigid3d(reshape(S.candidate_poses(i,1:9),[3,3]),S.candidate_poses(i,10:12));
        Ttauw = rigid3d(Twtau.Rotation',-Twtau.Translation*Twtau.Rotation');
        camera_extrinsics1 = Ttauw;
        camera_extrinsics2 = Tcw;
        cam_matrix1 = cameraMatrix(camera_intrinsics,camera_extrinsics1);
        cam_matrix2 = cameraMatrix(camera_intrinsics,camera_extrinsics2);
        [triangulated_point,reprojection_error,validity] = triangulate(double(S.first_observation(i,:)),double(S.candidate_keypoints(i,:)),cam_matrix1,cam_matrix2);
        if ~validity
            not_valid(i) = 1;
            S.candidate_keypoints_invalid_triangulation(i) = S.candidate_keypoints_invalid_triangulation(i)+1;
            continue
        end
        S.candidate_keypoints_invalid_triangulation(i) = 0;

        %determine angle between bearing vectors
        tau_ray = triangulated_point-Twtau.Translation;
        C_ray = triangulated_point-Twc.Translation;
        cos_alpha = dot(tau_ray,C_ray)/(norm(tau_ray)*norm(C_ray));

        %check whether landmark should be added
        if abs(cos_alpha)<=1 % just to be sure
            alpha = acos(cos_alpha);
            if abs(alpha)*180/pi>maxangle
                maxangle = abs(alpha)*180/pi;
            end
            if (abs(alpha)>=params.alpha_threshold) && ((reprojection_error < params.reprojection_error_threshold))
                %add as new landmark if the triangulated landmark isn't
                %negative and its reprojection error is smaller than a
                %threshold
                add_candidates(i) = 1;
                S.keypoints(end+1,:) = S.candidate_keypoints(i,:);
                S.landmarks(end+1,:) = triangulated_point;
                if params.BA == 1
                    global largest_index
                    S.indices(end+1) = largest_index+1;
                    largest_index = largest_index+1;
                end
                angle_too_small(i) = 0;
                reprojection_error_too_large(i) = 0;
            elseif ~(abs(alpha)>=params.alpha_threshold) && ~((reprojection_error < params.reprojection_error_threshold))
                angle_too_small(i) = 1;
                reprojection_error_too_large(i) = 1;
                S.candidate_keypoints_invalid_reprojection(i) = S.candidate_keypoints_invalid_reprojection(i)+1;
            elseif ~(abs(alpha)>=params.alpha_threshold) && ((reprojection_error < params.reprojection_error_threshold))
                angle_too_small(i) = 1;
                reprojection_error_too_large(i) = 0;
                S.candidate_keypoints_invalid_reprojection(i) = 0;
            elseif (abs(alpha)>=params.alpha_threshold) && ~((reprojection_error < params.reprojection_error_threshold))
                angle_too_small(i) = 0;
                reprojection_error_too_large(i) = 1;
                S.candidate_keypoints_invalid_reprojection(i) = S.candidate_keypoints_invalid_reprojection(i)+1;
            end
        end
    end
    disp(['candidates added: ',num2str(sum(add_candidates)),' / ',num2str(height(add_candidates))])  
    disp(['candidates not added due to bearing angle: ',num2str(sum(angle_too_small)),' / ',num2str(sum(angle_too_small))])  
    disp(['candidates not added due to reprojection error: ',num2str(sum(reprojection_error_too_large)),' / ',num2str(sum(reprojection_error_too_large))])  
    
    %remove added candidates from associated lists
    add_candidates = logical(add_candidates);
    angle_too_small = logical(angle_too_small);
    reprojection_error_too_large = logical(reprojection_error_too_large);
    not_valid = logical(not_valid);
    angle_too_small = logical(angle_too_small);
    S.first_observation(add_candidates,:) = [];
    S.candidate_keypoints(add_candidates,:) = [];
    S.candidate_poses(add_candidates,:) = [];
    S.candidate_keypoints_invalid_triangulation(add_candidates) = [];
    S.candidate_keypoints_invalid_reprojection(add_candidates) = [];
    angle_too_small(add_candidates,:) = [];
    reprojection_error_too_large(add_candidates,:) = [];
    not_valid(add_candidates,:) = [];
else
    angle_too_small = [];
    reprojection_error_too_large = [];
    not_valid = [];
end

%determine new keypoint candidates (filter out keypoints that are too similar to currently tracked keypoints or keypoint candidates)
keypoints = params.feature_detector(img,params.feature_quality,params.filter_size); %feature detector
first_observation_indicesx1 = ~ismembertol(keypoints.Location(:,1),S.keypoints(:,1),params.pixel_distance_threshold,'DataScale',1);
first_observation_indicesy1 = ~ismembertol(keypoints.Location(:,2),S.keypoints(:,2),params.pixel_distance_threshold,'DataScale',1);
if ~isempty(S.candidate_keypoints)
    first_observation_indicesx2 = ~ismembertol(keypoints.Location(:,1),S.candidate_keypoints(:,1),params.pixel_distance_threshold,'DataScale',1);
    first_observation_indicesy2 = ~ismembertol(keypoints.Location(:,2),S.candidate_keypoints(:,2),params.pixel_distance_threshold,'DataScale',1);
    first_observation_indices = (first_observation_indicesx1 | first_observation_indicesy1) & (first_observation_indicesx2 | first_observation_indicesy2);
else
    first_observation_indices = first_observation_indicesx1 | first_observation_indicesy1;
end

%draw new candidates to add based on a uniform distribution over the image
new_candidates = params.candidates_object(keypoints.Location(first_observation_indices,:));
first_observation_keypoints = selectUniform(new_candidates,params.select_uniform_threshold,size(img));
first_observation = first_observation_keypoints.Location;
first_observation_indeces = [zeros(height(S.candidate_keypoints),1);ones(height(first_observation),1)];
first_observation_indeces = logical(first_observation_indeces);

%add new keypoint candidates
S.first_observation = [S.first_observation;first_observation];
S.candidate_keypoints = [S.candidate_keypoints;first_observation];
S.candidate_keypoints_invalid_triangulation = [S.candidate_keypoints_invalid_triangulation;zeros(height(first_observation),1)];
S.candidate_keypoints_invalid_reprojection = [S.candidate_keypoints_invalid_reprojection;zeros(height(first_observation),1)];
disp(['new/current candidate keypoints: ',num2str(height(first_observation)),' / ',num2str(height(S.candidate_keypoints))])

%determine camera poses of keypoints at their first observation
pose = [reshape(Twc.Rotation,[1,9]),reshape(Twc.Translation,[1,3])];
candidate_poses = [repmat(pose,height(first_observation_keypoints),1),frame_num*ones(height(first_observation_keypoints.Location),1)];
S.candidate_poses = [S.candidate_poses;candidate_poses];

%plot results
plot_result(S,Twc,img,frame_num,angle_too_small,reprojection_error_too_large,first_observation_indeces,not_valid,params,last_frame)
toc

end