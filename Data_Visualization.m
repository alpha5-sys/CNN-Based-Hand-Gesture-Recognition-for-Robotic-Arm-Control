% Define the folder path to your CSV files
folderPath = 'C:\Users\Niklesh\Documents\Capstone Project\Project code\my_dataset_robotic_arm\001';

% Create a tabularTextDatastore to read all CSV files in the folder
ds = tabularTextDatastore(folderPath, 'FileExtensions', '.csv');

% Loop through each file in the datastore
while hasdata(ds)
    % Read the current file
    
    data = read(ds);

    % Extract the timestamp as x values (assuming first column contains timestamp)
    x = data{:, 1};
    
    % Get the column names for labeling
    columnNames = data.Properties.VariableNames;

    % Flex Sensor Data (flex_1 to flex_5)
    figure;
    hold on;
    for i = 3:7  % Flex sensor columns (flex_1 to flex_5)
        plot(x, data{:, i}, 'DisplayName', columnNames{i});
    end
    title('Flex Sensor Data');
    xlabel('Timestamp');
    ylabel('Flex Sensor Values');
    legend show;
    grid on;
    hold off;

    % Quaternion Data (Qw, Qx, Qy, Qz)
    figure;
    hold on;
    for i = 8:11  % Quaternion columns (Qw, Qx, Qy, Qz)
        plot(x, data{:, i}, 'DisplayName', columnNames{i});
    end
    title('Quaternion Data');
    xlabel('Timestamp');
    ylabel('Quaternion Values');
    legend show;
    grid on;
    hold off;

    % Gyroscope Data (GYRx, GYRy, GYRz)
    figure;
    hold on;
    for i = 12:14  % Gyroscope columns (GYRx, GYRy, GYRz)
        plot(x, data{:, i}, 'DisplayName', columnNames{i});
    end
    title('Gyroscope Data');
    xlabel('Timestamp');
    ylabel('Gyroscope Values');
    legend show;
    grid on;
    hold off;

    % Acceleration Data (ACCx, ACCy, ACCz, ACCx_body, ACCy_body, ACCz_body, ACCx_world, ACCy_world, ACCz_world)
    figure;
    hold on;
    for i = 15:23  % Acceleration columns (ACCx to ACCz_world)
        plot(x, data{:, i}, 'DisplayName', columnNames{i});
    end
    title('Acceleration Data');
    xlabel('Timestamp');
    ylabel('Acceleration Values');
    legend show;
    grid on;
    hold off;

    % Wait for the user input before moving to the next file
    userInput = input('Press Space to proceed to the next file, or press Enter to terminate...','s');
end


