% 加载MATLAB数据文件
load('TerrainData.mat');

% 查看变量列表
whos

% 直接指定高程数据变量名（根据whos输出结果）
elevationVarName = 'Final_Data';
elevation = evalin('base', elevationVarName);

% 显示数据基本信息
[rows, cols] = size(elevation);
fprintf('原始数据尺寸: %d行 x %d列\n', rows, cols);
fprintf('原始宽高比: %.2f\n', cols/rows);

% 目标比例
targetRatio = 16/5;  % 3.2

% 当前比例
currentRatio = cols/rows;

if currentRatio > targetRatio
    % 宽度过长，保持高度不变，裁剪宽度
    targetWidth = round(rows * targetRatio);
    startCol = round((cols - targetWidth) / 2);
    cropRegion = [startCol, 1, targetWidth, rows];  % [x, y, width, height]
else
    % 高度过长，保持宽度不变，裁剪高度
    targetHeight = round(cols / targetRatio);
    startRow = round((rows - targetHeight) / 2);
    cropRegion = [1, startRow, cols, targetHeight];  % [x, y, width, height]
end

fprintf('裁剪区域: [x=%d, y=%d, width=%d, height=%d]\n', cropRegion);
fprintf('裁剪后宽高比: %.2f\n', cropRegion(3)/cropRegion(4));

% 裁剪高程数据
croppedElevation = elevation(cropRegion(2):cropRegion(2)+cropRegion(4)-1, ...
                             cropRegion(1):cropRegion(1)+cropRegion(3)-1);

% 显示裁剪后数据尺寸
[newRows, newCols] = size(croppedElevation);
fprintf('裁剪后数据尺寸: %d行 x %d列\n', newRows, newCols);

% 保存裁剪后的数据到新的MATLAB文件
outputFile = 'TerrainData_cropped.mat';
save(outputFile, 'croppedElevation');
fprintf('已保存裁剪后的数据到: %s\n', outputFile);

% 可视化（可选）
figure;
subplot(1,2,1);
imagesc(elevation);
title('原始高程数据');
colorbar;

subplot(1,2,2);
imagesc(croppedElevation);
title('裁剪后高程数据 (16:5)');
colorbar;