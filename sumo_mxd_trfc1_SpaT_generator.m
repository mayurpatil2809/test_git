clc
clear all
% close all
import traci.constants

%% With varying ego injection time 
% Load the XML file
xDoc = xmlread(fullfile('traffic_sim_iter.rou.xml'));
allListItems = xDoc.getElementsByTagName('vehicle');
thisListItem = allListItems.item(0);
currentDepart = str2double(thisListItem.getAttribute('depart'));

%% With varying ego injection time 
for ii=1:30
    % Update the depart value
    newDepart = currentDepart + ii - 1;
    thisListItem.setAttribute('depart', num2str(newDepart));
    xmlwrite('traffic_sim_iter.rou.xml', xDoc);

    % To run simulation
    end_t = 7200; %end time for the sumo simulation
    dt = 1;   %simulation step time 0.01 seconds
    lastStep = end_t/dt; % number of simulation steps
    
    %variables
    currentStep = 1;
    ego_speed = 0;
    ego_velocity = 0;
    ego_x = 0;
    ego_y = 0;
    ego_yaw_angle = 0;
    integratorReset = 1;
    %loading the SUMO configuration
    traci.start(['sumo -c' '"' 'D:\OSU_Courses\PhD_2021\Learning\SUMO\routesampler_mixed_trfc_1\hvy_trfc/osm.sumocfg' '"']);
    %TLS=cell(1,4);
    %starting the simulink simulation and pausing so that it runs with 
    %loop for all simulations
    i = 1;
    k = 1;
    % Initialize TLS cell array to store traffic light information
    TLS = cell(lastStep, 21);
    % Initialize cell arrays to store traffic light values
    TLS_IDs = cell(1, lastStep);
    TLS_states = cell(1, lastStep);
    current_TLS_ID_array = {};
    current_light_state_array = {};
    while (currentStep <= lastStep)
        %Telling sumo to step once
        traci.simulationStep();
        
        %getting all the current vehicles in the simulation
        veh_ids = string(traci.vehicle.getIDList());
        veh_density(k) = length(veh_ids);
        k = k+1;
        %checking if the ego is in the current list of vehicles
        Check = contains(veh_ids,'ego');
        if find(Check==1)
            %getting the ego x and y position
            ego_pos = traci.vehicle.getPosition('ego');
            ego_x = ego_pos(1,1);
            ego_y = ego_pos(1,2);
    %         traci.vehicle.setSpeed('ego',50);
            ego_speed = traci.vehicle.getSpeed('ego');
            ESpeed(currentStep) = ego_speed;
            ego_distance = traci.vehicle.getDistance('ego');
            Edist(currentStep) = ego_distance;
            ego_waiting_time = traci.vehicle.getAccumulatedWaitingTime('ego');
            WT(currentStep) = ego_waiting_time;
            ego_yaw_angle = traci.vehicle.getAngle('ego');
            nextTLS = traci.vehicle.getNextTLS('ego');
            time = traci.simulation.getTime();
            if ~isempty(nextTLS)
                % Store the current ID and state
                current_TLS_ID = nextTLS{1, 1}{1, 1};
                current_light_state = nextTLS{1, 1}{1, 4};
                current_TLS_ID_array = [current_TLS_ID_array current_TLS_ID];
                SpaT_remaining(i) = traci.trafficlights.getNextSwitch(current_TLS_ID)-time;
            end
    
            % speed limit
            vehLaneID = traci.vehicle.getLaneID('ego');
            speed_limit = traci.lane.getMaxSpeed(vehLaneID);
            SL(currentStep) = speed_limit;
              
                
    %              strcmp(TLS{1,1}(1,1),{})
    %                 TLS={};
    %             end
            
            [lead_ID, D] = traci.vehicle.getLeader('ego',0);
            %getting all the lead vehicle information
            if D ~= -1
                lead_dist(i) = D;
                lead_pos = traci.vehicle.getPosition(lead_ID);
                lead_x(i) = lead_pos(1,1);
                lead_y(i) = lead_pos(1,2);
                lead_speed(i) = traci.vehicle.getSpeed(lead_ID);
            end
            
            %storing the traffic light values
            if size(nextTLS,2)>0   
                TLS_state(i) = convertCharsToStrings(current_light_state);
                distToNextLight(i) = nextTLS{1,1}{1,3};
                currentSize = size(nextTLS,2);
                for j=1:currentSize
                    TLS_state_list(j,i) = nextTLS{1,1}{1,4};
    %                 phase_duration1(j,i) = traci.simulation.getTime();
    %                 phase_duration2(j,i) = traci.trafficlights.getNextSwitch(nextTLS{1,1}{1,1});
    %                 PHASE_DURATION(j,i) = phase_duration2(j,i)-phase_duration1(j,i);
    %                 phase_duration(j,i) = round(PHASE_DURATION(j,i)*100)/100;
                     
                    switch TLS_state(i)
                        case 'G'
                             light_state(j,i) = 6;
                        case 'g'
                             light_state(j,i) = 6;
                        case 'y'
                            light_state(j,i) = 8;
                        case 'r'
                            light_state(j,i) = 3;
                    end 
                    
                end
             end
            
            %storing fuel consumption for ego
            ego_fuel(i) = traci.vehicle.getFuelConsumption('ego');
            simulation_time(i) = traci.simulation.getTime();
            
            i = i+1;
        end
    
        currentStep = currentStep + 1;
    end
    
    % set_param('traci_simulink_R19','SimulationCommand','Stop');
    traci.close(); 

    %
    new_TLS_state = cell(size(TLS_state));
    max_columns = size(SpaT_remaining, 2);
    
    new_SpaT_remaining = zeros(size(new_TLS_state, 1), max_columns);
    new_TLS_state_row = 0;
    previous_TLS_ID = '';
    % Iterate over the arrays
    for i = 1:numel(current_TLS_ID_array)
        current_TLS_ID = current_TLS_ID_array{i};
        
        % Check if a new TLS ID is detected
        if ~strcmp(current_TLS_ID, previous_TLS_ID)
            new_TLS_state_row = new_TLS_state_row + 1;
        end
        
        % Update the new_TLS_state
        new_TLS_state{new_TLS_state_row, i} = TLS_state{i};
        new_SpaT_remaining(new_TLS_state_row, i) = SpaT_remaining(:, i);
        % Update the previous_TLS_ID
        previous_TLS_ID = current_TLS_ID;
    end
    SpaT_num = size(new_SpaT_remaining,1);
    
    firstNonZeroIndex = find(ESpeed ~= 0, 1);
    ESpeed_new = ESpeed(firstNonZeroIndex:end);
    ESpeed_new = ESpeed_new(1:min(length(ESpeed_new), size(new_SpaT_remaining, 2)));
    ESpeed_new = ESpeed_new(1:size(new_SpaT_remaining, 2));

    % Coding TLS
    % Assuming new_TLS_state is the 21x1240 cell array containing the data
    
    % Create a new array for storing the converted values
    converted_TLS_state = zeros(size(new_TLS_state));
    
    % Replace 'G' or 'g' with 6
    converted_TLS_state(strcmp(new_TLS_state, 'G') | strcmp(new_TLS_state, 'g')) = 6;
    
    % Replace 'r' with 3
    converted_TLS_state(strcmp(new_TLS_state, 'r')) = 3;
    
    % Replace 'y' with 8
    converted_TLS_state(strcmp(new_TLS_state, 'y')) = 8;

    % Save SpaT to a .mat file
    SpaT_dur = new_SpaT_remaining;
    SpaT_state = converted_TLS_state;
    v_ego = ESpeed_new;
    SpaT_num = SpaT_num;
    save(fullfile('SpaT_profiles', ['SpaT_mixedRoute1_run_' num2str(ii) '.mat']), 'SpaT_dur', 'SpaT_state', 'v_ego', 'SpaT_num');
end