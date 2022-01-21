function [S,prev_S,Twc,Tcw,BA] = nViewBundleAdjustment(S,prev_S,Twc,BA,frame_num,validity_KLT,inlier_indeces,camera_intrinsics,params)
    
%update keypoint and landmark indices
S.indices = prev_S.indices(validity_KLT);
S.indices = S.indices(inlier_indeces);
prev_S.indices = prev_S.indices(validity_KLT,:); %just for completeness; not needed
prev_S.indices = prev_S.indices(inlier_indeces,:); %just for completeness; not needed

%update BA table with current keypoints, landmarks and pose estimation
ViewId = uint32(frame_num);
Orientation = {Twc.Rotation};
Location = {Twc.Translation};
Keypoints = {S.keypoints};
Landmarks = {S.landmarks};
Indices = {S.indices};
global largest_index;
largest_index_temp = largest_index;
largest_index = 0;
BA = [BA;table(ViewId,Orientation,Location,Keypoints,Landmarks,Indices,largest_index)];
largest_index = largest_index_temp;
BA.largest_index(end-1) = largest_index;
if size(BA,1) > params.BA_n_views
    BA = BA(2:end,:);
end

%determine keypoint pointtracks and extract corresponding landmarks
unique_indices = unique(cell2mat(BA.Indices));
n_unique_indices = size(unique_indices,1);
pointTracks(n_unique_indices) = pointTrack();
ba_landmarks = zeros(n_unique_indices,3);
for i = 1:size(unique_indices,1)
    this_index = unique_indices(i); 
    ViewIDs = [];
    keypoints = [];
    landmarks = [];
    for j = 1:size(BA.ViewId,1)
        keypoints_this_frame = cell2mat(BA.Keypoints(j));
        landmarks_this_frame = cell2mat(BA.Landmarks(j));
        cell_row = find(cell2mat(BA.Indices(j))==this_index,1);
        if ~isempty(cell_row)
            ViewIDs(end+1) = BA.ViewId(j);
            keypoints(end+1,:) = keypoints_this_frame(cell_row,:);
            ba_landmarks(i,:) = landmarks_this_frame(cell_row,:);
        end
    end
    pointTracks(i) = pointTrack(ViewIDs,keypoints,this_index);
end
ba_cameraPoses = [BA(:,'ViewId'),BA(:,'Orientation'),BA(:,'Location')];

disp(['-----------------------------------------------------------'])
%run bundle adjustment
%do not update initial pose as it coindcides with world frame origin
if BA.ViewId(1) == 0
    [ba_landmarks,ba_cameraPoses] = bundleAdjustment(ba_landmarks,pointTracks,ba_cameraPoses,camera_intrinsics,'FixedViewIDs',0,'AbsoluteTolerance',params.BA_absolute_tolerance,'PointsUndistorted',true,'Verbose',true);
else
    [ba_landmarks,ba_cameraPoses] = bundleAdjustment(ba_landmarks,pointTracks,ba_cameraPoses,camera_intrinsics,'AbsoluteTolerance',params.BA_absolute_tolerance,'PointsUndistorted',true,'Verbose',true);
end
disp(['-----------------------------------------------------------'])

%update BA table with the bundle adjusted landmarks and poses
for j = 1:size(BA,1)
   landmarks_this_frame = cell2mat(BA.Landmarks(j));
   for i = 1:size(unique_indices,1)
       this_index = unique_indices(i); 
       cell_row = find(cell2mat(BA.Indices(j))==this_index,1);
       if ~isempty(cell_row)
           landmarks_this_frame(cell_row,:) = ba_landmarks(i,:);
       end
   end
   BA.Landmarks(j) = {landmarks_this_frame};
end
if ~isempty(prev_S.candidate_poses)
    %update poses of first candidate observations with bundle adjusted
    %poses
    candidate_poses_indices = uint32(prev_S.candidate_poses(:,end));
    unique_candidate_poses_indices = unique(candidate_poses_indices);
    for i = 1:size(unique_candidate_poses_indices)
        this_frame_Id = unique_candidate_poses_indices(i);
        BA_row = find(BA.ViewId==this_frame_Id);
        if ~isempty(BA_row)
            array_rows = find(prev_S.candidate_poses(:,end)==this_frame_Id);
            ba_pose = [reshape(cell2mat(BA.Orientation(BA_row)),[1,9]),reshape(cell2mat(BA.Location(BA_row)),[1,3]),double(this_frame_Id)];
            prev_S.candidate_poses(array_rows,:) = repmat(ba_pose,height(array_rows),1);
        end
    end
end
S.landmarks = cell2mat(BA.Landmarks(end));
prev_S.landmarks = cell2mat(BA.Landmarks(end-1));
BA.Orientation = ba_cameraPoses.Orientation;
BA.Location = ba_cameraPoses.Location;
Twc = rigid3d(cell2mat(BA.Orientation(end)),cell2mat(BA.Location(end)));
Tcw = rigid3d(Twc.Rotation',-Twc.Translation*Twc.Rotation');

%update plot trajectory and landmarks with bundle adjusted poses and
%landmarks
global full_trajectory
global full_landmarks
for i = 0:params.BA_n_views-1
    if height(full_trajectory)-i>0 && height(BA.Location) > i+1
        location_this_frame = cell2mat(BA.Location(end-i-1));
        landmarks_this_frame = cell2mat(BA.Landmarks(end-i-1));
        full_trajectory(end-i,:) = location_this_frame;
        full_landmarks{end-i} = landmarks_this_frame;
    end
end

end