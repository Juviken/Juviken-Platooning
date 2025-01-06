% MATLAB Script to load CSV data and prepare for System Identification Toolbox

% Specify the file name
file_name = 'vehicle_1_immediate_data.csv';  % Replace with your actual file path

% Load the data from the CSV file
data_table = readtable(file_name);

% Extract relevant columns (stamp, pwm, and velocity)
time_stamps = data_table.stamp;  % Time stamps
pwm = data_table.pwm;            % Input signal (PWM)
velocity = data_table.velocity;  % Output signal (Velocity)

% Remove rows with missing data (NaN)
valid_idx = ~isnan(pwm) & ~isnan(velocity);
time_stamps = time_stamps(valid_idx);
pwm = pwm(valid_idx);
velocity = velocity(valid_idx);

% Convert time stamps to relative time (starting at zero)
relative_time = time_stamps - time_stamps(1);  % Start time at 0
Ts = mean(diff(relative_time));                % Sampling time in seconds

% Create time-series data for iddata
sys_id_data = iddata(velocity, pwm, Ts);

% Save time stamps as a separate variable for reference
time_data = relative_time;  % Relative time in seconds

% Save the iddata object and time data to a .mat file
save('sys_id_data_with_time.mat', 'sys_id_data', 'time_data');

% Display a message
disp('Data has been processed and saved as sys_id_data_with_time.mat');
