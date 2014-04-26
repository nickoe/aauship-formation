% Create a cell array of file names of calibration images.
    for i = 1:10
%         imageFileName = sprintf('image%02d.jpg', i);
        imageFileName = sprintf('GOPR3938.JPG', i);
        imageFileNames{i} = fullfile(matlabroot, 'toolbox', 'vision','visiondemos', 'calibration', 'fishEye', imageFileName);
    end

% Detect calibration pattern.
    [imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

% Generate world coordinates of the corners of the squares.
    squareSize = 29; % square size in millimeters
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
    cameraParameters = estimateCameraParameters(imagePoints, worldPoints);

% Remove lens distortion and display results
    I = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondemos', 'calibration', 'fishEye', 'image01.jpg'));
    J = undistortImage(I, cameraParameters);

% Display original and corrected image.
    figure; imshowpair(I, J, 'montage');
    title('Original Image (left) vs. Corrected Image (right)');