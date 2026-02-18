"""
Main code of SUAVPy
"""

import threading
import traci
import csv
import numpy as np
import ujson as json
import socket
import os
import tkinter as tk
from tkinter import messagebox, ttk, Toplevel, Label
from _utils import Calculations
import copy

import time 


def non_blocking_warning(title, message):
    warning_window = Toplevel()
    warning_window.title(title)
    warning_window.geometry("300x80")
    warning_window.configure(bg='#2E2E2E')
    #icon_path = "images/kiosLogo.ico"
    #warning_window.iconbitmap(icon_path)
    
    
    label = Label(warning_window, text=message, bg='#2E2E2E', bd =5, fg='#FFFFFF', font=('Helvetica', 10))
    label.pack(pady=10)

    warning_window.after(5000, warning_window.destroy)  # Automatically close after 5 seconds
    warning_window.attributes("-topmost", True) 
    

def get_uav_position_input():
    root = tk.Tk()
    
    root.title("Local GUI")
    
    root.withdraw()

    # Set the window icon
    #icon_path = "images/kiosLogo.ico"
    #root.iconbitmap(icon_path)
    
    root.geometry("400x250")
    #root.resizable(False, False)

    # Apply a dark theme
    root.configure(bg='#2E2E2E')

    style = ttk.Style()
    style.theme_use('alt')  # Using 'alt' as a base theme for customization

    # Configure the style to match a dark theme
    style.configure('TButton', font=('Helvetica', 11), padding=6, background='#2E2E2E', foreground='#FFFFFF')
    style.configure('TLabel', font=('Helvetica', 11), padding=5, background='#2E2E2E', foreground='#FFFFFF')
    style.configure('TEntry', font=('Helvetica', 11), padding=5, fieldbackground='#3C3F41', foreground='#FFFFFF')

    # Create a dialog window for input
    dialog = tk.Toplevel(root)
    dialog.configure(bg='#2E2E2E')
    #dialog.iconbitmap(icon_path)
    
    dialog.attributes("-topmost", True)

    # UAV ID input
    uav_id_label = tk.Label(dialog, text="Enter UAV ID:", bg='#2E2E2E', fg='#FFFFFF', font=('Helvetica', 11))
    uav_id_label.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)  # Centered with 'nsew' sticky option
    uav_id_entry = tk.Entry(dialog, bg='#3C3F41', fg='#FFFFFF', insertbackground='#FFFFFF', font=('Helvetica', 11))
    uav_id_entry.grid(row=0, column=1, padx=5, pady=5)

    # UAV position input
    position_label = tk.Label(dialog, text="Enter UAV position as \n (t, x, y, z, φ):", bg='#2E2E2E', fg='#FFFFFF', font=('Arimo', 11))
    position_label.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)  # Centered with 'nsew' sticky option
    position_entry = tk.Entry(dialog, bg='#3C3F41', fg='#FFFFFF', insertbackground='#FFFFFF', font=('Helvetica', 11))
    position_entry.grid(row=1, column=1, padx=5, pady=5)

    # Button frame
    button_frame = tk.Frame(dialog, bg='#2E2E2E')
    button_frame.grid(row=2, columnspan=2, pady=20)

    # Submit button (green)
    submit_button = tk.Button(button_frame, text="Submit", bg='#4CAF50', fg='#FFFFFF', activebackground='#45A049', font=('Helvetica', 11), command=root.quit)
    submit_button.grid(row=0, column=0, padx=10)

    # Break button (red) that sends 'break'
    break_button = tk.Button(button_frame, text="Break", bg='#F44336', fg='#FFFFFF', activebackground='#E57373', font=('Helvetica', 11), command=lambda: (uav_id_entry.insert(0, "break"), root.quit()))
    break_button.grid(row=0, column=1, padx=10)

    
    root.mainloop()

    uav_id_value = uav_id_entry.get()
    position_value = position_entry.get()

    root.destroy()
    return uav_id_value, position_value


class UAVSimulation:

    def __init__(self, config_file):
        self.read_config(config_file)
        self.timing_data = {}
        self.stop_flag = False
        self.calc = Calculations(self.uav_speed, self.simulation_step_length, self.yaw_speed)
        self.uav_positions_list, self.time_list, self.uav_yaw_angles_list = self.uav_path_data()
        self.uav_home = {uav_id: self.uav_positions_list[uav_id][0] for uav_id in range(self.num_UAVs)}

    def read_config(self, config_file):
        try:
            with open(config_file, 'r') as file:
                config = json.load(file)
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file {config_file} not found.")
        except json.JSONDecodeError:
            raise ValueError("Configuration file is not a valid JSON.")
        
        self.UavModel = config['Uav Model']
        self.GuiOption = config['GUI Option']
        self.battery_mode = config.get('Battery Mode', False)
        self.num_UAVs = config['Number of UAVs']
        self.UavMode = config.get('Uav Mode', 'Hovering')
        
        self.mode = config.get('Input Mode', 'Offline')
        self.movement = config.get('Movement', 'Continuous')
        
        self.server_option = config.get('Remote Server', False)
        self.local_gui = config.get('Local GUI', False)
        
        self.delay_option = config.get('Delay', 0 )
        
        self.simulation_step_length = float(config['Step length (s)'])
        self.total_simulation_steps = int(config['Total time (s)'] / self.simulation_step_length)
        
        if self.UavModel == 'Mavic 2e':
            self.fov_degrees = [68.0643, 40.0455]
            t_h, t_c, t_m = 29*60, 31*60, 21.6*60 # tiempo de autonomia en: hovering, velocidad crucero y maxima velocidad respectivamente
            self.v_cruise = 7.0 #velocidad crucero, 25 km/h
            self.uav_speed = 13.8 #velocidad actual del uav, se usará para calcular la descarga de batería
            self.v_max = 13.8 #velocidad maxima, 50 km/h
            self.yaw_speed = 10
            self.battery_life = int(1500 / self.simulation_step_length) # 25 minutes 
        elif self.UavModel == 'Mini 3 pro':
            self.fov_degrees = [66.9161, 40.2499]
            t_h, t_c, t_m = 30*60, 34*60, 25*60 # tiempo de autonomia en: hovering, velocidad crucero y maxima velocidad respectivamente
            self.v_cruise = 6.0 #velocidad crucero, 21.6 km/h
            self.uav_speed = 6.0 #velocidad actual del uav, se usará para calcular la descarga de batería
            self.v_max = 12.0 #velocidad maxima, 43.2 km/h
            self.yaw_speed = 10
            self.battery_life = 600 # 30 minutes 
        elif self.UavModel == 'Manual':
            self.fov_degrees = list(map(float, config['FOV (deg)']))
            t_h = config.get('Hovering autonomy (s)', 1800) # default 30 minutes
            t_c = config.get('Cruise autonomy (s)', 2000) # default 33.3 minutes
            t_m = config.get('Max velocity autonomy (s)', 1500) # default 25 minutes
            self.uav_speed = float(config['UAV Speed'])
            self.v_cruise = float(config.get('UAV Cruise Speed', self.uav_speed))
            self.v_max = float(config.get('UAV Max Speed', self.uav_speed))
            self.yaw_speed = float(config['Yaw Speed'])
            self.battery_life = int(config.get('Battery life (s)',1800)/self.simulation_step_length) # 30 minutes if not stated 
        else:
            raise ValueError('UAV model does not exist')
        
        self.hover_rate, self.cruise_rate, self.max_rate = self.calculate_uav_rates(t_h, t_c, t_m)

        self.network_file = config['Network file']
        self.sumocfg_file = config['Sumocfg file']
        
        if not os.path.exists(self.sumocfg_file):
            raise FileNotFoundError(f"SUMO configuration file {self.sumocfg_file} not found.")
        if not os.path.exists(self.network_file):
            raise FileNotFoundError(f"SUMO configuration file {self.network_file} not found.")
    
        self.uav_data = config['uav_data']
        # Keep a copy of the original data for reference
        self.original_uav_data = copy.deepcopy(self.uav_data)  
        
        if self.server_option:
            self.uav_data = {str(i): [[0] * 5] for i in range(self.num_UAVs)}
            
        
        print(f' Number of Uavs: {self.num_UAVs} \n Uav Model: {self.UavModel} \n Uav Speed: {self.uav_speed} m/s \n Yaw Speed: {self.yaw_speed} \n Battery Life: {self.battery_life/60*self.simulation_step_length} minutes ')
    

    def calculate_uav_rates(self, t_hover, t_cruise, t_max):
        hover_rate = 1.0
        # A velocidad crucero será < 1.0 (ahorro)
        cruise_rate = t_hover / t_cruise if t_cruise > 0 else 1.0
        # A velocidad máxima, el rate será > 1.0 (gasto)
        max_rate = t_hover / t_max if t_max > 0 else 1.0
    
        return hover_rate, cruise_rate, max_rate
    
    def calculate_discharge_rate(self, velocity):
        if velocity <= self.v_cruise:
            ratio = velocity / self.v_cruise
            return self.hover_rate + ratio * (self.cruise_rate - self.hover_rate)
        else:
            # Usamos v_max_ref para asegurar que el max_rate se aplique correctamente
            v_norm = (velocity - self.v_cruise) / (self.v_max - self.v_cruise)
            return self.cruise_rate + (self.max_rate - self.cruise_rate) * (v_norm ** 3)

        
    def start_sumo(self):         
        sumo_cmd = ['sumo-gui' if self.GuiOption else 'sumo', "-c", 
                    self.sumocfg_file, 
                    "--step-length", str(self.simulation_step_length),
                    "--delay", str(self.delay_option),
                    "--start", "true",
                    "--quit-on-end", "True"]
                    #"--mesosim", "True"]
                    #, "--edgedata-output", 'Outputs/edgeData.xml',
                    #"--fcd-output",'Outputs/fcd.xml']

        traci.start(sumo_cmd)
        #if self.GuiOption: // Potential Update
            #traci.gui.setZoom("View #0", 50)
            #traci.gui.setOffset("View #0", 1145, 150)
            #traci.gui.setZoom("View #0", 600) 
            #traci.gui.setOffset("View #0", 1900, 1700)
        print("TraCI is started")

        self.create_uav_vehicles()
        traci.simulationStep()
        print("Vehicles after first step:", traci.vehicle.getIDList(), flush=True)
        


    def create_uav_vehicles(self):
        edges = traci.edge.getIDList()
        if not edges:
            raise RuntimeError("No edges in SUMO network")

        dummy_edge = edges[20]

        if "dummy" not in traci.route.getIDList():
            traci.route.add("dummy", [dummy_edge])

        for uav_id in range(self.num_UAVs):
            veh_id = f"uav{uav_id}"

            traci.vehicle.add(
                vehID=veh_id,
                routeID="dummy",
                typeID="uav_type",
                depart="now",
                departPos="free",
                departSpeed=0,
            )


            traci.vehicle.setSpeed(veh_id, 0)
            traci.vehicle.setSpeedMode(veh_id, 0)
            traci.vehicle.setLaneChangeMode(veh_id, 0)
            print(f"Vehículo {veh_id} añadido exitosamente.")

            
            
    def run_simulation(self, output_file='Outputs/uav_output.csv'):
        
        polygon_ids = [f"fov_polygon_{i}" for i in range(self.num_UAVs)]
        border_polygon_ids = [f"fov_border_polygon_{i}" for i in range(self.num_UAVs)]
        poi_ids = [f"uav_poi_{i}" for i in range(self.num_UAVs)]
        icon_paths = {'Manual': "images/manualLQ.png",
                      'Mini 3 pro': "images/mini3proLQ.png",
                      'Mavic 2e': "images/mavic2e.png"}
    
        icon_path = icon_paths.get(self.UavModel, "images/manualLQ.png")
        self.battery_life_steps = int(self.battery_life)
        # Initialize battery_life_steps based on the first time value in uav_data
        battery_life_steps = {str(uav_id): max(0, self.uav_data[str(uav_id)][0][0]) for uav_id in range(self.num_UAVs)}
        polygon_exists = {i: False for i in range(self.num_UAVs)}
        poi_exists = {i: False for i in range(self.num_UAVs)}
        #Flag used to avoid multiple warnings for the same UAV when it reaches the battery warning threshold
        warning_sent = {str(i): False for i in range(self.num_UAVs)}
        #Flag used to avoid multiple warnings for the same UAV when it returns to the base
        return_sent = {str(i): False for i in range(self.num_UAVs)}
        #Flag used to avoid multiple warnings for the same UAV when it reaches the battery life threshold and is removed from the simulation
        stop_sent = {str(i): False for i in range(self.num_UAVs)}
        #List to keep track of active UAVs, used to stop the simulation when all UAVs are removed due to battery life end
        uavs_activos = [i for i in range(self.num_UAVs)]
        #State of the UAV, used to determine if it is on a mission,returning to base or charging
        uav_state = {str(i): "MISSION" for i in range(self.num_UAVs)}
        charging_time_steps = int(200 / self.simulation_step_length)
        charging_counter = {str(i): 0 for i in range(self.num_UAVs)}
        # Track the original-path time at which the UAV left its mission
        departure_time = {str(i): 0 for i in range(self.num_UAVs)}

        
        with open(output_file, mode='w', newline='') as file:
            writer = csv.writer(file, delimiter=',')
            writer.writerow(['Step', 'Seconds', 'UAV_ID', 'UAV_X', 'UAV_Y', 'UAV_Z', 'Yaw', 'VehicleID', 'X', 'Y', 'Speed'])
    
            step = 0
    
            if self.local_gui:
                user_input_thread = threading.Thread(target=self.get_user_input)
                user_input_thread.start()
                
            if self.server_option:
                server_thread = threading.Thread(target=self.start_server)
                server_thread.start()
                    
            while step < self.total_simulation_steps and not self.stop_flag:
                
                traci.simulationStep()
                step += 1
                for uav_id in range(self.num_UAVs):
                    if uav_state[str(uav_id)] == "CHARGING":

                        charging_counter[str(uav_id)] += 1

                        # Keep vehicle at home position so SUMO doesn't lose track of it
                        home = self.uav_home[uav_id]
                        veh_id = f"uav{uav_id}"
                        if veh_id in traci.vehicle.getIDList():
                            traci.vehicle.moveToXY(
                                vehID=veh_id,
                                edgeID="",
                                laneIndex=-1,
                                x=home[0],
                                y=home[1],
                                keepRoute=2
                            )

                        if charging_counter[str(uav_id)] >= charging_time_steps:
                            battery_life_steps[str(uav_id)] = 0
                            warning_sent[str(uav_id)] = False
                            return_sent[str(uav_id)] = False
                            stop_sent[str(uav_id)] = False
                            non_blocking_warning("Charging Complete", f"UAV {uav_id} has finished charging and is back on a mission.")
                            # Resume original path from where the UAV left off, time-shifted to now
                            original_data = copy.deepcopy(self.original_uav_data[str(uav_id)])
                            dep_t = departure_time[str(uav_id)]
                            current_time = step * self.simulation_step_length
                            # Keep only waypoints at or after the departure time
                            remaining = [p for p in original_data if p[0] >= dep_t]
                            if not remaining:
                                remaining = [original_data[-1]]
                            # Shift timestamps so the first remaining waypoint starts at current_time
                            time_offset = current_time - remaining[0][0]
                            for point in remaining:
                                point[0] += time_offset
                            # Remove waypoints beyond simulation end
                            total_time = self.total_simulation_steps * self.simulation_step_length
                            remaining = [p for p in remaining if p[0] <= total_time]
                            # Ensure last waypoint reaches simulation end
                            if remaining and remaining[-1][0] < total_time:
                                last = list(remaining[-1])
                                last[0] = total_time
                                remaining.append(last)
                            # Prepend home position at t=0 so path covers all steps
                            home_pos = self.uav_home[uav_id]
                            first_yaw = remaining[0][4] if remaining else 0
                            remaining.insert(0, [0, home_pos[0], home_pos[1], home_pos[2], first_yaw])
                            self.uav_data[str(uav_id)] = remaining
                            self.uav_positions_list, self.time_list, self.uav_yaw_angles_list = self.uav_path_data()
                            uav_state[str(uav_id)] = "MISSION"
            
                    if uav_state[str(uav_id)] == "RETURNING":
                            home = self.uav_home[uav_id]
                            curr_pos_xy = traci.vehicle.getPosition(f"uav{uav_id}")
                            curr_pos = np.array(curr_pos_xy)
                            home_xy = np.array([home[0], home[1]])
                            distance = np.linalg.norm(curr_pos - home_xy)
                            #print(f"UAV {uav_id} returning to base. Current position: {curr_pos}, Home position: {home}")
                            #print(f"Distance to home: {distance}")
                            if distance < 0.5:
                                uav_state[str(uav_id)] = "CHARGING"
                                charging_counter[str(uav_id)] = 0
                                non_blocking_warning("Charging", f"UAV {uav_id} charging...")

                    veh_id = f"uav{uav_id}"
                    
                    # Check if step is within bounds of available positions
                    # if step >= len(self.uav_positions_list[uav_id]):
                    #     print(f"[WARN] {veh_id} no tiene posición para step {step}, se omite moveToXY", flush=True)
                    #     continue
                    
                    #init_pos = self.uav_positions_list[uav_id][step]
                    times = self.time_list[uav_id]
                    traj = self.uav_positions_list[uav_id]

                    if step in times:
                        idx = times.index(step)
                        init_pos = traj[idx]
                    else:
                        continue
                    x_real, y_real = init_pos[0], init_pos[1]

                    if veh_id not in traci.vehicle.getIDList():
                        print(f"[WARN] {veh_id} no existe en step {step}, se omite moveToXY", flush=True)
                        continue


                    traci.vehicle.moveToXY(
                    vehID=veh_id,
                    edgeID="dummy",
                    laneIndex=0,
                    x=x_real,
                    y=y_real,
                    keepRoute=2
                    )  


                if step == 1:
                    print("Vehicles after first step:", traci.vehicle.getIDList(), flush=True)


                print(traci.vehicle.getPosition("uav0"))
                print(traci.vehicle.getPosition("uav1"))
                print(traci.vehicle.getPosition("uav2"))
                print("\n")
                if step == 650:
                    print("Vehicles after first step:", traci.vehicle.getIDList(), flush=True)
                # if step == 1700:
                #     print("Vehicles after first step:", traci.vehicle.getIDList(), flush=True)

                # if step == 5:
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")
                #     #print("Vehicles after 5 step:", traci.vehicle.getIDList(), flush=True)

                # if step == 50:
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")
                #     #print("Vehicles after 50 step:", traci.vehicle.getIDList(), flush=True)
                # if step == 140:
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 150:
                #     print("150")   
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")
                #     print("Vehicles after 150 step:", traci.vehicle.getIDList(), flush=True)

                # if step == 160:
                #     print("160")
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 168:
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")
                
                # if step == 150:
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 180:
                #     print("180")
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 200:
                #     print("200")
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 600:
                #     print("600")
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 602:
                #     print("600")
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")

                # if step == 800:
                #     print(traci.vehicle.getPosition("uav0"))
                #     print(traci.vehicle.getPosition("uav1"))
                #     print(traci.vehicle.getPosition("uav2"))
                #     print("\n")
                
    
                for veh_id in traci.simulation.getDepartedIDList():
                    traci.vehicle.subscribe(veh_id, [traci.constants.VAR_POSITION, traci.constants.VAR_SPEED])
                subscribed_data = traci.vehicle.getAllSubscriptionResults()

    
                for uav_id, (uav_positions, times, uav_yaw_angles) in enumerate(zip(self.uav_positions_list, self.time_list, self.uav_yaw_angles_list)):
                    
                    if step == 1 and self.GuiOption:
                        uav_position = uav_positions[0]
                        yaw_angle = uav_yaw_angles[0]
                        field_of_view_size = self.calc.fov_calculation(self.fov_degrees, uav_position[2])
                        self.calc.add_fov_polygon(uav_position, field_of_view_size, yaw_angle, polygon_ids[uav_id], border_polygon_ids[uav_id])
                        self.calc.add_poi(poi_ids[uav_id], uav_position, yaw_angle, icon_path)
                        polygon_exists[uav_id] = True
                        poi_exists[uav_id] = True
                    
                    if self.battery_mode and step < len(times) and uav_positions[step][2] > 0:

                        if step > 0:
                            pos_now = np.array(uav_positions[step])
                            pos_prev = np.array(uav_positions[step-1])
                            distance= np.linalg.norm(pos_now - pos_prev)
                            actual_velocity= distance/ self.simulation_step_length
                        else:
                            actual_velocity = 0
                        
                        #Discharge rate based on especifications and current velocity
                        discharge_rate = self.calculate_discharge_rate(actual_velocity)
                        #print(discharge_rate)

                        battery_life_steps[str(uav_id)] += discharge_rate
    
                        if battery_life_steps[str(uav_id)] >= self.battery_life_steps - (500 / self.simulation_step_length) and not warning_sent[str(uav_id)]:
                            #print(f" \n Warning: Uav: {uav_id} has 5 minutes of battery left \n ")
                            battery_percentage =  500 / self.battery_life_steps * 100
                            non_blocking_warning("Battery Warning", f"Warning: UAV {uav_id} has {battery_percentage} percentage of battery left.")
                            warning_sent[str(uav_id)] = True
                        if battery_life_steps[str(uav_id)] >= self.battery_life_steps - (200 / self.simulation_step_length) and not return_sent[str(uav_id)]:
                            if uav_state[str(uav_id)] == "MISSION":
                                uav_state[str(uav_id)] = "RETURNING"
                                # Remember where in the original path we left off
                                departure_time[str(uav_id)] = step * self.simulation_step_length
                                non_blocking_warning("Battery Warning", f"Warning: Returning to base")
                                home = self.uav_home[uav_id]
                                t_now = step * self.simulation_step_length
                                curr_pos = uav_positions[step]
                                curr_angle = uav_yaw_angles[step]
                                self.uav_data[str(uav_id)] = [
                                    [t_now, curr_pos[0], curr_pos[1], curr_pos[2], curr_angle],
                                    [t_now + 1, home[0], home[1], home[2], 0]
                                ]
                                self.uav_positions_list, self.time_list, self.uav_yaw_angles_list = self.uav_path_data()
                                return_sent[str(uav_id)] = True

                        if battery_life_steps[str(uav_id)] >= self.battery_life_steps and not stop_sent[str(uav_id)]:
                            if polygon_exists[uav_id]:
                                self.calc.remove_fov_polygon(polygon_ids[uav_id], border_polygon_ids[uav_id])
                                polygon_exists[uav_id] = False
                            if poi_exists[uav_id]: 
                                self.calc.remove_poi(poi_ids[uav_id])
                                poi_exists[uav_id] = False
                            if veh_id in traci.vehicle.getIDList():
                                traci.vehicle.remove(f"uav{uav_id}")
                                uavs_activos.remove(uav_id)
                            if not uavs_activos:
                                    self.stop_flag = True
                            #print(f" \n Uav: {uav_id} lost signal \n")
                            stop_sent[str(uav_id)] = True
                            non_blocking_warning("Signal Lost", f"UAV {uav_id} lost signal.")
                            #messagebox.showwarning("Signal Lost", f"UAV {uav_id} lost signal.")                       
                            continue
                   
                    if step in times:
                        index = times.index(step)
                        if index < len(uav_positions) and index < len(uav_yaw_angles):
                            uav_position = uav_positions[index]
                            yaw_angle = uav_yaw_angles[index]
                            field_of_view_size = self.calc.fov_calculation(self.fov_degrees, uav_position[2])
                            
                            
                            # reduce the long if statements.
                            moved = index > 0 and (np.any(np.array(uav_positions[index - 1]) != np.array(uav_position)) or uav_yaw_angles[index - 1] != yaw_angle)
                            update_required = self.GuiOption and moved
    
                            if self.GuiOption:
                                if polygon_exists[uav_id] and update_required:
                                    self.calc.update_fov_polygon(uav_position, field_of_view_size, yaw_angle, polygon_ids[uav_id], border_polygon_ids[uav_id])
                                if step % 1 == 0 and poi_exists[uav_id] and update_required: ## PERFORMANCE CHECK ##
                                    self.calc.update_poi(poi_ids[uav_id], uav_position, yaw_angle)
    
                            if self.UavMode == 'Sampling':
                                if moved:
                                    if polygon_exists[uav_id]:
                                        self.calc.remove_fov_polygon(polygon_ids[uav_id], border_polygon_ids[uav_id])
                                        polygon_exists[uav_id] = False
                                elif not polygon_exists[uav_id]:
                                    self.calc.add_fov_polygon(uav_position, field_of_view_size, yaw_angle, polygon_ids[uav_id], border_polygon_ids[uav_id])
                                    polygon_exists[uav_id] = True
                                    
                            # REMOVE OR ADD FOR CONSECUTIVE UAV POSITIONS           
                            writer.writerow([step, step * self.simulation_step_length, uav_id, uav_position[0], uav_position[1], uav_position[2], yaw_angle, '', '', '', ''])
    
                            if not (self.UavMode == 'Sampling' and moved):
                                vehicles_info = self.calc.get_vehicles_in_fov(subscribed_data, uav_position, field_of_view_size, yaw_angle, return_info=('positions', 'speeds'))
                                vehicles_in_view = vehicles_info['vehicle_ids']
                                positions_in_view = vehicles_info['positions']
                                speeds_in_view = vehicles_info['speeds']
                                     
                                for vehicle_id, position, speed in zip(vehicles_in_view, positions_in_view, speeds_in_view):
                                    writer.writerow([step, step * self.simulation_step_length, uav_id, uav_position[0], uav_position[1], uav_position[2], yaw_angle, vehicle_id, position[0], position[1], speed])

            if self.local_gui:
                self.stop_flag = True
                user_input_thread.join()
                
            if self.server_option:
                self.stop_flag = True
                server_thread.join()
            

        
        traci.close()
        print("TraCI is closed")
        

    

        
    def start_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('localhost', 1024))
        server_socket.listen(1)
        print("Python server is waiting for a connection...")
        
        try:
            conn, addr = server_socket.accept()
            with conn:
                print(f"Connected by {addr}")
                while not self.stop_flag:
                    try:
                        data = conn.recv(1024)
                        if not data:
                            continue
                        data = data.decode()
                        print(f"Received data from client: {data}")
                        response = f"Data received: {data}"
                        conn.sendall(response.encode())
                        
                        # parse data and update UAV path
                        uav_id, details = data.split(":")
                        uav_id = int(uav_id.strip())
                        details = json.loads(details.strip())
                        if uav_id in range(self.num_UAVs):
                            self.update_uav_path(uav_id, details)
                    except (ConnectionResetError, ConnectionAbortedError) as e:
                        print(f"Connection error: {e}")
                        break
                    except Exception as e:
                        print(f"An error occurred: {e}")
                        break
        finally:
            server_socket.close()
            print("Server socket closed")

    
    def get_user_input(self):
        while not self.stop_flag:
            uav_id, position_str = get_uav_position_input()
    
            # Check if the break button was pressed
            if uav_id.strip().lower() == 'break':
                self.stop_flag = True
                break
    
            try:
                uav_id = int(uav_id.strip())
                position = json.loads(position_str.strip())
                if uav_id in range(self.num_UAVs):
                    self.update_uav_path(uav_id, position)
                else:
                    messagebox.showerror("Invalid Input", "The UAV ID is out of range.")
            except ValueError:
                messagebox.showerror("Invalid Input", "Please enter a valid UAV ID and position format.")
            except json.JSONDecodeError:
                messagebox.showerror("Invalid JSON Format", "Please ensure the position is in the correct JSON format.")



    def update_uav_path(self, uav_id, details):
        # Here I ignore one waypoint after the new entry
        
        time, x, y, z, yaw_angle = details
    
        # Check if local GUI is true and modify the path accordingly
        if self.local_gui:
            if str(uav_id) in self.uav_data:
                # Find the index of the next waypoint to be executed
                next_index = next((idx for idx, point in enumerate(self.uav_data[str(uav_id)]) if point[0] > time), None)
                
                # Insert the new waypoint
                self.uav_data[str(uav_id)].insert(next_index, [time, x, y, z, yaw_angle])
                
                # Remove the next waypoint in the sequence
                if next_index is not None and len(self.uav_data[str(uav_id)]) > next_index + 1:
                    self.uav_data[str(uav_id)].pop(next_index + 1)
    
        # Otherwise, proceed as before
        else:
            if str(uav_id) in self.uav_data:
                self.uav_data[str(uav_id)].append([time, x, y, z, yaw_angle])
                self.uav_data[str(uav_id)] = sorted(self.uav_data[str(uav_id)])  # sort the UAV data
    
        # Recalculate paths with new input
        self.uav_positions_list, self.time_list, self.uav_yaw_angles_list = self.uav_path_data()
        


    def uav_path_data(self):
        uav_positions_list = []
        time_list = []
        uav_yaw_angles_list = []
    
        for uav_id in range(self.num_UAVs):
            data_points = self.uav_data[str(uav_id)]
            
            positions = [[point[1], point[2], point[3]] for point in data_points]
            yaw_angles = [point[4] for point in data_points]
            times = [int(point[0] / self.simulation_step_length) for point in data_points]
    
            if self.movement == 'Discrete': # and self.server_updated_paths:
                # when server or script is on we do not follow the simple dynamics (rotate-hover-rotate)
                uav_positions_list.append(positions)
                time_list.append(times)
                uav_yaw_angles_list.append(yaw_angles)
            else:
                uav_positions = []
                step_times = []
                uav_yaw_angles = []
                total_steps = times[0]  # start of counting
    
                for i in range(len(positions) - 1):
                    init_pos = np.array(positions[i])
                    next_pos = np.array(positions[i + 1])
                    current_yaw_angle = yaw_angles[i]
                    target_yaw_angle = yaw_angles[i + 1]
                    next_step = times[i + 1]
    
                    move_yaw_angle = self.calc.calculate_yaw_angle(init_pos, next_pos) # angle between the two 'move' positions  
                    move_steps = self.calc.calculate_move_steps(init_pos, next_pos) # steps needed to move from A to B
                    rotate_to_move_steps = self.calc.calculate_rotate_steps(move_yaw_angle - current_yaw_angle) # steps needed to rotate before start moving
                    rotate_to_target_steps =  self.calc.calculate_rotate_steps(target_yaw_angle - move_yaw_angle) # steps needed to rotate after reaching position but not yaw angle
                        
                    while total_steps < next_step: # HOVER PARALLELS
                        uav_positions.append(init_pos)
                        step_times.append(total_steps)
                        if self.UavMode == 'Hovering' or self.UavMode == 'Sampling':
                            uav_yaw_angles.append(current_yaw_angle)
                        elif self.UavMode == 'Spinning':
                            current_yaw_angle += self.yaw_speed * self.simulation_step_length
                            uav_yaw_angles.append(current_yaw_angle)
                        total_steps += 1
                        
                        
                    if self.UavMode == 'Spinning':
                        current_yaw_angle = current_yaw_angle % 360  # fast spinning fix.
                        rotate_to_move_steps = self.calc.calculate_rotate_steps(move_yaw_angle - current_yaw_angle) 
                    
                    if np.array_equal(init_pos[:2], next_pos[:2]):                
                        # No horizontal movement, only change in height or yaw
                        rotate_to_target_steps = self.calc.calculate_rotate_steps(target_yaw_angle - current_yaw_angle)
                        move_steps = self.calc.calculate_move_steps(init_pos, next_pos)                  
                        
                        # Move to the new height, no yaw change
                        for step in range(move_steps):
                            position = init_pos + (next_pos - init_pos) * (step + 1) / move_steps
                            uav_positions.append(position)
                            step_times.append(total_steps)
                            uav_yaw_angles.append(current_yaw_angle)  # Keep the same yaw while changing height
                            total_steps += 1                        
                        
                        for step in range(rotate_to_target_steps):
                            yaw = current_yaw_angle + (target_yaw_angle - current_yaw_angle) * (step + 1) / rotate_to_target_steps
                            uav_positions.append(next_pos)
                            step_times.append(total_steps)
                            uav_yaw_angles.append(yaw)
                            total_steps += 1
                            
                    else:
                        for step in range(rotate_to_move_steps):
                            yaw = current_yaw_angle + (move_yaw_angle - current_yaw_angle) * (step + 1) / rotate_to_move_steps
                            uav_positions.append(init_pos)
                            step_times.append(total_steps)
                            uav_yaw_angles.append(yaw)
                            total_steps += 1
        
                        for step in range(move_steps):
                            position = init_pos + (next_pos - init_pos) * (step + 1) / move_steps
                            uav_positions.append(position)
                            step_times.append(total_steps)
                            uav_yaw_angles.append(move_yaw_angle) # uav_yaw_angles.append(current_yaw_angle)
                            total_steps += 1
        
                        if self.UavMode != 'Spinning':
                            for step in range(rotate_to_target_steps):
                                yaw = move_yaw_angle + (target_yaw_angle - move_yaw_angle) * (step + 1) / rotate_to_target_steps
                                uav_positions.append(next_pos)
                                step_times.append(total_steps)
                                uav_yaw_angles.append(yaw)
                                total_steps += 1

                final_pos = np.array(positions[-1])
                final_yaw_angle = yaw_angles[-1]
    
                # hovering at the final position until the end of simulation
                while total_steps < self.total_simulation_steps:
                    uav_positions.append(final_pos)
                    step_times.append(total_steps)
                    if self.UavMode =='Spinning':
                        final_yaw_angle += self.yaw_speed * self.simulation_step_length
                    uav_yaw_angles.append(final_yaw_angle)
                    total_steps += 1
    
                uav_positions_list.append(uav_positions)
                time_list.append(step_times)
                uav_yaw_angles_list.append(uav_yaw_angles)
    
        return uav_positions_list, time_list, uav_yaw_angles_list


    
        
def start_simulation_thread(sim, tk_root):
    try:
        sim.start_sumo()
        sim.run_simulation()
    except traci.exceptions.FatalTraCIError:
        print("Simulation terminated due to SUMO closing.")
    finally:
        try:
            traci.close()
        except traci.exceptions.FatalTraCIError:
            print("TraCI was already closed.")
        # Stop Tkinter main loop once simulation finishes
        tk_root.quit()  # Stops the Tkinter loop

if __name__ == "__main__":
    #Initialize Tkinter main window
    root = tk.Tk()
    root.withdraw()  # Hide the main window since we don't need it

    # Initialize the simulation
    sim = UAVSimulation('config.json')

    # Start the simulation in a separate thread so it doesn't block the Tkinter event loop
    simulation_thread = threading.Thread(target=start_simulation_thread, args=(sim, root))
    simulation_thread.start()

    # Start the Tkinter event loop (this must run in the main thread)
    root.mainloop()

    # Wait for the simulation thread to finish
    simulation_thread.join()

    # Exit the program once everything is done
    print("Simulation and UI closed.")

#######
#######
#######

# if __name__ == "__main__":
    
#     sim = UAVSimulation('config.json')
    
#     try:
#          t0 = time.time()
#          sim.start_sumo()
#          sim.run_simulation()
#          t1 = time.time()
#          print(f"Elapsed time: {t1 - t0} seconds")
#     except traci.exceptions.TraCIException:
#          traci.close()
#          print("TraCI is fully closed from exception")

#######
#######
#######
        

