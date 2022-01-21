function plot_result(S,Twc,image,frame_num,angle_too_small,reprojection_error_too_large,first_observation_indeces,not_valid,params,last_frame)

persistent fig
if isempty(fig)
    fig = figure;
    fig.WindowState = 'maximized';
    pause(0.1);
else
    fig.WindowState = 'maximized';
end
set(gcf,'color','w');
font_size_title = 15;
font_size_labels = 15;
font_size_tick_labels = 15;
font_size_legend = 15;
n_poses = params.past_poses_plotted;
clf

%plot current image and corresponding keypoints
subplot(2,5,[1 2 3]);
imshow(image)
hold on
plot(S.keypoints(:,1),S.keypoints(:,2),'go')
plot(S.candidate_keypoints(:,1),S.candidate_keypoints(:,2),'rx')
if sum(angle_too_small) ~= 0
    plot(S.candidate_keypoints(angle_too_small,1),S.candidate_keypoints(angle_too_small,2),'bo')
else
    plot(NaN,NaN,'bo')
end
if sum(reprojection_error_too_large) ~= 0
    plot(S.candidate_keypoints(reprojection_error_too_large,1),S.candidate_keypoints(reprojection_error_too_large,2),'cs')
else
    plot(NaN,NaN,'cs')
end
if sum(first_observation_indeces) ~= 0
    plot(S.candidate_keypoints(first_observation_indeces,1),S.candidate_keypoints(first_observation_indeces,2),'md')
else
    plot(NaN,NaN,'md')
end
if sum(not_valid) ~= 0
    plot(S.candidate_keypoints(not_valid,1),S.candidate_keypoints(not_valid,2),'k*')
else
    plot(NaN,NaN,'k*')
end
legend('all keypoints','all candidates','angle too small','repr. err. too large','first observation', 'triangulation invalid','Location','bestoutside','FontSize',font_size_legend)
legend('boxoff')  
title(append('current frame: no. ',num2str(frame_num)),'FontSize',font_size_title)
ax = gca;
ax.FontSize = font_size_tick_labels; 

%plot number of landmarks 
global num_landmarks
if size(num_landmarks) <= n_poses
    num_landmarks(end+1) = size(S.landmarks(:,1),1);
else
    num_landmarks = num_landmarks(2:end);
    num_landmarks(end+1) = size(S.landmarks(:,1),1);
end
subplot(2,5,6)
xlim([-n_poses 0])
plot(-size(num_landmarks,2)+1:0,num_landmarks,'b-')
xlabel('preceding frames','FontSize',font_size_labels)
ylabel('\# tracked landmarks','FontSize',font_size_labels)
title("\# tracked landmarks (last " +n_poses+ " frames)", 'FontSize',font_size_title)
ax = gca;
ax.FontSize = font_size_tick_labels; 

%plot full trajectory
global full_trajectory
full_trajectory(end+1,:) = Twc.Translation;
subplot(2,5,7)
plot(full_trajectory(:,1),full_trajectory(:,3),'b-')
title('full trajectory','FontSize',font_size_title)
axis equal
xlabel('$x$ coordinate','FontSize',font_size_labels)
ylabel('$z$ coordinate','FontSize',font_size_labels)
ax = gca;
ax.FontSize = font_size_tick_labels; 

%plot full trajectory with landmarks
global full_landmarks
global full_landmark_mat
full_landmarks{end+1} = rmoutliers(S.landmarks,'percentiles',[0 params.percentile_all_landmarks]);
subplot(2,5,8)
if size(full_landmarks,2) > 1
    full_landmark_mat(end+1:end+height(cell2mat(full_landmarks(end))),:) = cell2mat(full_landmarks(end));
    plot(full_landmark_mat(:,1),full_landmark_mat(:,3),'k.','color',[0.75,0.75,0.75]);
end
hold on
plot(full_trajectory(:,1),full_trajectory(:,3),'b-')
hold off
axis equal
xlabel('$x$ coordinate','FontSize',font_size_labels)
ylabel('$z$ coordinate','FontSize',font_size_labels)
title('full trajectory with landmarks','FontSize',font_size_title)
ax = gca;
ax.FontSize = font_size_tick_labels; 

%plot trajectory over last n_poses frames
subplot(2,5,[4 5 9 10]);
axis equal
plot(full_trajectory(max(1,end-(n_poses-1)):end,1),full_trajectory(max(1,end-(n_poses-1)):end,3),'bx')
hold on
current_landmarks = S.landmarks(:,[1,3]);
dist_landmarks = current_landmarks - Twc.Translation([1,3]);
dist_landmarks = dist_landmarks(:,1).^2+dist_landmarks(:,2).^2;
[~, idx] = mink(dist_landmarks, params.n_current_landmarks_plotted);
current_landmarks = current_landmarks(idx,:);
if params.BA == 1
    plot(full_trajectory(max(1,end-params.BA_n_views+1):end,1),full_trajectory(max(1,end-params.BA_n_views+1):end,3),'rx')
    plot(current_landmarks(:,1),current_landmarks(:,2),'k.','color',[0.5,0.5,0.5])
    legend('past trajectory',['trajectory in BA window ($n = $ ',num2str(params.BA_n_views),')'],['closest ',num2str(params.n_current_landmarks_plotted),' landmarks'],'FontSize',font_size_legend)
    legend('boxoff')  
else
    plot(full_trajectory(end,1),full_trajectory(end,3),'rx')
    plot(current_landmarks(:,1),current_landmarks(:,2),'k.','color',[0.5,0.5,0.5])
    legend('past trajectory','current location',['closest ',num2str(params.n_current_landmarks_plotted),' landmarks'],'FontSize',font_size_legend)
    legend('boxoff')  
end
hold off
axis equal
xlabel('$x$ coordinate','FontSize',font_size_labels)
ylabel('$z$ coordinate','FontSize',font_size_labels)
title("trajectory (last " +n_poses + " frames - closest " +params.n_current_landmarks_plotted+ " landmarks)",'FontSize',font_size_title)
ax = gca;
ax.FontSize = font_size_tick_labels; 

sgtitle('\bfseries{Visual Odometry Mini Project - Robert John, Nicola Loi, Vincent van der Brugge, Jonas Walther}','FontSize',17) 

%save results
if not(isfolder('results'))
    mkdir('results')
end
if not(isfolder('results/KITTI'))
    mkdir('results/KITTI')
end
if not(isfolder('results/Malaga'))
    mkdir('results/Malaga')
end
if not(isfolder('results/Parking'))
    mkdir('results/Parking')
end

if frame_num == last_frame
    if params.BA == 0
        if params.ds == 0
            save('results/KITTI/full_trajectory_KITTI.mat','full_trajectory')
            save('results/KITTI/full_landmarks_KITTI.mat','full_landmarks')
        elseif params.ds == 1
            save('results/Malaga/full_trajectory_Malaga.mat','full_trajectory')
            save('results/Malaga/full_landmarks_Malaga.mat','full_landmarks')
        elseif params.ds == 2
            save('results/Parking/full_trajectory_Parking.mat','full_trajectory')
            save('results/Parking/full_landmarks_Parking.mat','full_landmarks')
        end
    elseif params.BA == 1
        if params.ds == 0
            save('results/KITTI/BA_full_trajectory_KITTI.mat','full_trajectory')
            save('results/KITTI/BA_full_landmarks_KITTI.mat','full_landmarks')
        elseif params.ds == 1
            save('results/Malaga/BA_full_trajectory_Malaga.mat','full_trajectory')
            save('results/Malaga/BA_full_landmarks_Malaga.mat','full_landmarks')
        elseif params.ds == 2
            save('results/Parking/BA_full_trajectory_Parking.mat','full_trajectory')
            save('results/Parking/BA_full_landmarks_Parking.mat','full_landmarks')
        end
    end
end

end