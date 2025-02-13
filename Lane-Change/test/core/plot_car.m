function[] = plot_car(state, L, H, color)
    % Extract car position and orientation
    theta = state(3); % Orientation (in radians)
    center1 = state(1); % X position
    center2 = state(2); % Y position

    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Define car vertices (relative to center)
    X = [-L/2, L/2, L/2, -L/2]; % X coordinates
    Y = [-H/2, -H/2, H/2, H/2]; % Y coordinates

    % Rotate and translate vertices
    for i = 1:4
        T(:, i) = R * [X(i); Y(i)];
    end

    % Calculate global coordinates
    x_lower_left = center1 + T(1, 1);
    x_lower_right = center1 + T(1, 2);
    x_upper_right = center1 + T(1, 3);
    x_upper_left = center1 + T(1, 4);

    y_lower_left = center2 + T(2, 1);
    y_lower_right = center2 + T(2, 2);
    y_upper_right = center2 + T(2, 3);
    y_upper_left = center2 + T(2, 4);

    % Combine coordinates
    x_coor = [x_lower_left, x_lower_right, x_upper_right, x_upper_left];
    y_coor = [y_lower_left, y_lower_right, y_upper_right, y_upper_left];

    % Plot the car
    patch('Vertices', [x_coor; y_coor]', 'Faces', [1, 2, 3, 4], ...
          'Edgecolor', color, 'Facecolor', color, 'Linewidth', 2.0);

    % Maintain aspect ratio
    axis equal;
end