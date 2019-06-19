function [transformed] = custom_icp(moving_path, target_path)

disp('[INFO] Starting transformation.');
moving = pcread(moving_path);
target = pcread(target_path);
tform = pcregrigid(moving, target);
transformed = pctransform(moving, tform);
subplot(2, 2, 1); pcshow(moving); title('Moving Point Cloud');
subplot(2, 2, 2); pcshow(target); title('Target Point Cloud');
subplot(2, 2, 3); pcshow(transformed); title('Final Moved Point Cloud');
subplot(2, 2, 4); pcshow(target); hold; pcshow(transformed); title('Overlapped result');
disp('[INFO] Transformation Completed!');
