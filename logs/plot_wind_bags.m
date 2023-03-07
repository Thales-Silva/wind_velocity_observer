function plot_bags(filepath)

    bag = rosbag(filepath);

    wind_velocity = select(bag, 'topic', 'firefly/wind_speed');
    wind_velocity_msg = readMessages(wind_velocity, 'DataFormat', 'struct');

    t_wind_sec = cellfun(@(m) double(m.Header.Stamp.Sec), wind_velocity_msg);
    t_wind_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), wind_velocity_msg);
    x_wind_velocity = cellfun(@(m) double(m.Velocity.X), wind_velocity_msg);
    y_wind_velocity = cellfun(@(m) double(m.Velocity.Y), wind_velocity_msg);
    z_wind_velocity = cellfun(@(m) double(m.Velocity.Z), wind_velocity_msg);
    t_wind = t_wind_sec + t_wind_nsec * 1e-9;



    observed_wind_velocity = select(bag, 'topic', 'estimated_wind_velocity');
    observed_wind_velocity_msg = readMessages(observed_wind_velocity, 'DataFormat', 'struct');

    t_obs_sec = cellfun(@(m) double(m.Header.Stamp.Sec), observed_wind_velocity_msg);
    t_obs_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), observed_wind_velocity_msg);
    x_observed_wind_velocity = cellfun(@(m) double(m.Vector.X), observed_wind_velocity_msg);
    y_observed_wind_velocity = cellfun(@(m) double(m.Vector.Y), observed_wind_velocity_msg);
    z_observed_wind_velocity = cellfun(@(m) double(m.Vector.Z), observed_wind_velocity_msg);

    t_obs = t_obs_sec + t_obs_nsec * 1e-9;
    t_wind = t_wind - t_obs(1);
    t_obs = t_obs - t_obs(1);

    figure(1),
    hold on;
    grid on;
    xlabel('t [secs]');
    ylabel('v_{w} [m/s]');

    plot(t_wind, x_wind_velocity, '--', 'Color', 'r', 'LineWidth', 1.0);
    plot(t_wind, y_wind_velocity, '--', 'Color', 'g', 'LineWidth', 1.0);
    plot(t_wind, z_wind_velocity, '--', 'Color', 'b', 'LineWidth', 1.0);

    plot(t_obs, x_observed_wind_velocity, '-', 'Color', 'r', 'LineWidth', 1.0);
    plot(t_obs, y_observed_wind_velocity, '-', 'Color', 'g', 'LineWidth', 1.0);
    plot(t_obs, z_observed_wind_velocity, '-', 'Color', 'b', 'LineWidth', 1.0);

    aux_vec = [x_wind_velocity;
               y_wind_velocity;
               z_wind_velocity;
               x_observed_wind_velocity;
               y_observed_wind_velocity;
               z_observed_wind_velocity];

    axis([0 max(t_obs) min(aux_vec) max(aux_vec)]);

    legend('$$\boldmath{u}$$', '$$\boldmath{v}$$', '$$\boldmath{w}$$', '$$\boldmath{\hat{u}}$$','$$\boldmath{\hat{v}}$$','$$\boldmath{\hat{w}}$$','Interpreter','Latex');

end
