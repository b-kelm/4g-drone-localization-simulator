import streamlit as st
import pandas as pd
import numpy as np
import folium
from folium.plugins import MarkerCluster
from scipy.optimize import least_squares
import random
import math
import time

# --- Configuration ---
DRONE_EMOJI = "✈️"
# Adjust drone icon size to accommodate error text below it
DRONE_ICON_SIZE = (40, 40) # Larger height for text

CELL_TOWER_EMOJI = "🗼"
CELL_TOWER_ICON_SIZE = (25, 25)

# Radius for loading and displaying towers around the drone
LOAD_RADIUS_KM = 10
LOAD_RADIUS_M = LOAD_RADIUS_KM * 1000

# Limit the number of green dotted lines to closest towers
MAX_DISPLAYED_CONNECTION_LINES = 5

# Trilateration specific limits
TRILATERATION_MIN_RSSI_DBM = -90 # Only towers with RSSI >= -90 dBm are considered for trilateration
MAX_TRILATERATION_TOWERS = 7 # Max number of strongest towers used for trilateration

# --- Session State Initialization (MUST happen at the top level) ---
# Default map center (Bielefeld, Germany)
DEFAULT_INITIAL_MAP_CENTER = [52.02, 8.53]

# Initialize all session state variables
if 'full_cell_towers_df' not in st.session_state:
    st.session_state['full_cell_towers_df'] = None
if 'last_uploaded_filename' not in st.session_state:
    st.session_state['last_uploaded_filename'] = None

if 'drone_lat_lon' not in st.session_state:
    st.session_state['drone_lat_lon'] = DEFAULT_INITIAL_MAP_CENTER
if 'last_clicked_lat' not in st.session_state:
    st.session_state['last_clicked_lat'] = None
if 'last_clicked_lon' not in st.session_state:
    st.session_state['last_clicked_lon'] = None
if 'drone_map_click' not in st.session_state: 
    st.session_state['drone_map_click'] = {'lat': None, 'lon': None}

if 'recalculate_flag' not in st.session_state:
    st.session_state['recalculate_flag'] = True 

if 'cell_towers_df_filtered' not in st.session_state:
    st.session_state['cell_towers_df_filtered'] = pd.DataFrame()
if 'm_folium_obj' not in st.session_state:
    st.session_state['m_folium_obj'] = None
if 'connected_towers_info' not in st.session_state: 
    st.session_state['connected_towers_info'] = []
if 'estimated_drone_lat_lon' not in st.session_state:
    st.session_state['estimated_drone_lat_lon'] = None
if 'localization_error_m' not in st.session_state:
    st.session_state['localization_error_m'] = None


# --- Helper Functions ---

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance

def calculate_distance_df(df, drone_lat, drone_lon):
    R = 6371000 
    phi1 = np.radians(drone_lat)
    phi2 = np.radians(df['lat'])
    delta_phi = np.radians(df['lat'] - drone_lat)
    delta_lambda = np.radians(df['lon'] - drone_lon)

    a = np.sin(delta_phi / 2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    distance = R * c
    return distance

def calculate_rssi(distance_m, attenuation_level_db, frequency_mhz=1800, tx_power_dbm=40):
    if distance_m < 1: 
        distance_m = 1

    distance_km = distance_m / 1000.0
    
    path_loss_db = 32.45 + 20 * np.log10(frequency_mhz) + 20 * np.log10(distance_km)
    
    rssi_dbm = tx_power_dbm - path_loss_db - attenuation_level_db
    
    min_dbm = -110
    max_dbm = -50
    
    percentage = max(0, min(100, (rssi_dbm - min_dbm) / (max_dbm - min_dbm) * 100))
    return rssi_dbm, percentage

def estimate_drone_position(connected_towers_data, drone_rssi_readings, current_overall_attenuation_db): 
    if len(connected_towers_data) < 3:
        return None, "Not enough connected towers for trilateration (need at least 3)."

    tower_coords = []
    distances_from_rssi = []
    weights = [] 

    def rssi_to_distance(rssi_dbm, attenuation_db_param, frequency_mhz=1800, tx_power_dbm=40): 
        path_loss_db = tx_power_dbm - attenuation_db_param - rssi_dbm
        
        if path_loss_db <= 0:
            return 1 
        
        try:
            distance_km = 10**((path_loss_db - 32.45 - 20 * np.log10(frequency_mhz)) / 20)
        except RuntimeWarning:
             return 1000000 
        except OverflowError:
             return 1000000
        
        return distance_km * 1000

    for tower in connected_towers_data: 
        tower_coords.append([tower['lat'], tower['lon']])
        
        actual_rssi_dbm = None
        for reading in drone_rssi_readings:
            if reading['tower_id'] == tower['cell']: 
                actual_rssi_dbm = reading['rssi_dbm']
                break
        
        if actual_rssi_dbm is not None:
            est_dist = rssi_to_distance(actual_rssi_dbm, current_overall_attenuation_db) 
            distances_from_rssi.append(est_dist)
            
            weight = max(0.01, tower['rssi_percentage'] / 100.0) 
            weights.append(weight)
        else:
            distances_from_rssi.append(1000000)
            weights.append(0.01) 

    tower_coords = np.array(tower_coords)
    distances_from_rssi = np.array(distances_from_rssi)
    weights = np.array(weights)

    def residuals(point_xy, tower_coords_ll, distances_m, weights_arr):
        res = []
        for i in range(len(tower_coords_ll)):
            lat_tower, lon_tower = tower_coords_ll[i]
            calculated_distance = haversine(point_xy[0], point_xy[1], lat_tower, lon_tower)
            res.append(weights_arr[i] * (calculated_distance - distances_m[i]))
        return np.array(res)

    initial_guess_lat = np.mean(tower_coords[:, 0])
    initial_guess_lon = np.mean(tower_coords[:, 1])

    try:
        result = least_squares(residuals, [initial_guess_lat, initial_guess_lon], 
                               args=(tower_coords, distances_from_rssi, weights),
                               bounds=([-90,-180], [90,180]))

        estimated_lat, estimated_lon = result.x
        return (estimated_lat, estimated_lon), None
    except Exception as e:
        return None, f"Trilateration failed: {e}. Check tower data and signal strength."


# --- Streamlit App ---
st.set_page_config(layout="wide", page_title="Cellular Localization Drone Simulator")

st.sidebar.title("Simulator Settings")

# --- Input/Upload of OpenCellID .csv Data ---
st.sidebar.header("0) Cell Tower Data")
uploaded_file = st.sidebar.file_uploader("Upload OpenCellID CSV", type="csv")
st.sidebar.markdown("[OpenCellID Database Format](https://wiki.opencellid.org/wiki/Database_format)")


if uploaded_file is not None:
    if st.session_state['full_cell_towers_df'] is None or uploaded_file.name != st.session_state['last_uploaded_filename']:
        with st.spinner(f"Loading {uploaded_file.name}... This might take a moment for large files."):
            try:
                column_names = [
                    'radio', 'mcc', 'net', 'area', 'cell', 'unit', 'lon', 'lat', 
                    'range', 'samples', 'changeable', 'created', 'updated', 'averageSignal'
                ]
                df_temp = pd.read_csv(uploaded_file, names=column_names, header=0) 
                
                df_temp['lon'] = pd.to_numeric(df_temp['lon'], errors='coerce')
                df_temp['lat'] = pd.to_numeric(df_temp['lat'], errors='coerce')
                df_temp = df_temp.dropna(subset=['lat', 'lon'])
                df_temp['cell'] = df_temp['cell'].astype(str)

                st.session_state['full_cell_towers_df'] = df_temp
                st.session_state['last_uploaded_filename'] = uploaded_file.name
                st.sidebar.success(f"Loaded {len(st.session_state['full_cell_towers_df'])} total cell towers into memory.")
                st.session_state['recalculate_flag'] = True 
            except Exception as e:
                st.sidebar.error(f"Error loading CSV: {e}")
                st.session_state['full_cell_towers_df'] = None
                st.session_state['last_uploaded_filename'] = None
                st.session_state['recalculate_flag'] = False 
else:
    st.sidebar.info("Please upload an OpenCellID CSV file to populate towers.")

# Default towers if no file is uploaded (for quick testing)
if st.session_state['full_cell_towers_df'] is None or st.session_state['full_cell_towers_df'].empty:
    st.sidebar.warning("Using default example towers. Upload a CSV for real data.")
    example_towers_data = {
        'radio': ['LTE', 'LTE', 'GSM', 'UMTS'],
        'mcc': [262, 262, 262, 262],
        'net': [1, 1, 1, 1],
        'area': [12345, 12345, 12346, 12347],
        'cell': ['10001', '10002', '10003', '10004'],
        'unit': [10, 20, np.nan, 30],
        'lon': [8.5307, 8.5350, 8.5200, 8.5280],
        'lat': [52.0210, 52.0250, 52.0180, 52.0230],
        'range': [5000, 4500, 6000, 5500],
        'samples': [100, 90, 110, 80],
        'changeable': [0, 0, 0, 0],
        'created': [1678886400, 1678886400, 1678886400, 1678886400],
        'updated': [1709424000, 1709424000, 1709424000, 1709424000],
        'averageSignal': [-70, -75, -65, -80]
    }
    st.session_state['full_cell_towers_df'] = pd.DataFrame(example_towers_data)
    st.session_state['recalculate_flag'] = True


# --- Noise level ---
st.sidebar.header("1) Noise Level")
constant_noise_offset = st.sidebar.slider("Constant Noise Offset (dB, random per tower)", 0.0, 10.0, 2.0, 0.1)
time_noise_magnitude = st.sidebar.slider("Time-Dependant Noise Magnitude (dB)", 0.0, 5.0, 1.0, 0.1)
time_noise_frequency = st.sidebar.slider("Time-Dependant Noise Frequency (Hz)", 0.01, 1.0, 0.1, 0.01)

# --- Overall Attenuation level ---
st.sidebar.header("2) Overall Attenuation")
overall_attenuation_db = st.sidebar.slider("Overall Signal Attenuation (dB)", 0.0, 50.0, 10.0, 0.1)

# --- Slider with % of inoperative cell towers ---
st.sidebar.header("3) Inoperative Towers")
inoperative_percentage = st.sidebar.slider("Percentage of Inoperative Cell Towers (%)", 0, 100, value=10, step=1)

# --- Aircraft Location Input (Text Fields) ---
st.sidebar.header("4) Set Drone Location")

def update_drone_from_input():
    # Only update if input values are valid numbers and different from current drone location
    try:
        new_lat = float(st.session_state.lat_input_key)
        new_lon = float(st.session_state.lon_input_key)
        
        # Check for significant change to avoid unnecessary reruns
        if haversine(st.session_state['drone_lat_lon'][0], st.session_state['drone_lat_lon'][1], new_lat, new_lon) > 0.5:
            st.session_state['drone_lat_lon'] = [new_lat, new_lon]
            st.session_state['recalculate_flag'] = True
    except ValueError:
        st.sidebar.warning("Please enter valid numbers for Latitude and Longitude.")

# Use st.number_input for better input control and validation
st.sidebar.number_input(
    "Latitude",
    value=float(st.session_state['drone_lat_lon'][0]), # Cast to float for slider default
    format="%.4f",
    key="lat_input_key",
    on_change=update_drone_from_input,
    step=0.0001
)
st.sidebar.number_input(
    "Longitude",
    value=float(st.session_state['drone_lat_lon'][1]), # Cast to float for slider default
    format="%.4f",
    key="lon_input_key",
    on_change=update_drone_from_input,
    step=0.0001
)

# --- Update Calculation Button ---
st.sidebar.markdown("---")
if st.sidebar.button("Update Calculation"):
    st.session_state['recalculate_flag'] = True


# --- Main Calculation Logic (conditional on recalculate_flag) ---
if st.session_state['recalculate_flag']:
    st.session_state['recalculate_flag'] = False # Reset flag immediately

    # --- Filter cell towers by drone's current radius AND LTE type ---
    current_drone_lat, current_drone_lon = st.session_state['drone_lat_lon']
    
    if st.session_state['full_cell_towers_df'] is not None and not st.session_state['full_cell_towers_df'].empty:
        df_temp_filtered = st.session_state['full_cell_towers_df'].copy()
        
        # Filter for LTE towers only
        df_temp_filtered = df_temp_filtered[df_temp_filtered['radio'] == 'LTE']
        
        df_temp_filtered['distance_to_drone'] = calculate_distance_df(
            df_temp_filtered, current_drone_lat, current_drone_lon
        )
        
        st.session_state['cell_towers_df_filtered'] = df_temp_filtered[
            df_temp_filtered['distance_to_drone'] <= LOAD_RADIUS_M
        ].copy()
        
        num_inoperative = int(len(st.session_state['cell_towers_df_filtered']) * (inoperative_percentage / 100))
        if num_inoperative > len(st.session_state['cell_towers_df_filtered']):
            num_inoperative = len(st.session_state['cell_towers_df_filtered'])
        
        inoperative_indices = np.random.choice(st.session_state['cell_towers_df_filtered'].index, num_inoperative, replace=False)
        
        st.session_state['cell_towers_df_filtered']['inoperative'] = False
        st.session_state['cell_towers_df_filtered'].loc[inoperative_indices, 'inoperative'] = True
        
        st.info(f"Loaded {len(st.session_state['cell_towers_df_filtered'])} LTE towers within {LOAD_RADIUS_KM} km radius for display.")
    else:
        st.session_state['cell_towers_df_filtered'] = pd.DataFrame() 
        st.warning("No cell tower data available. Please upload a CSV.")

    # Create the Folium map object for this calculation cycle
    m = folium.Map(location=[current_drone_lat, current_drone_lon], zoom_start=14, tiles="CartoDB positron")

    # Add cell towers to map
    if not st.session_state['cell_towers_df_filtered'].empty:
        for idx, tower in st.session_state['cell_towers_df_filtered'].iterrows():
            grayscale_style = "filter: grayscale(100%);" if tower['inoperative'] else ""
            tower_icon = folium.DivIcon(
                html=f'<div style="font-size: {CELL_TOWER_ICON_SIZE[0]}px; text-align: center; line-height: 1; {grayscale_style}">{CELL_TOWER_EMOJI}</div>',
                icon_size=CELL_TOWER_ICON_SIZE,
                icon_anchor=(CELL_TOWER_ICON_SIZE[0] / 2, CELL_TOWER_ICON_SIZE[1] / 2)
            )
            folium.Marker(
                location=[tower['lat'], tower['lon']],
                icon=tower_icon,
                tooltip=f"ID: {tower['cell']}<br>MCC: {tower['mcc']}<br>Net: {tower['net']}<br>Area: {tower['area']}<br>Radio: {tower['radio']}<br>Dist: {tower['distance_to_drone']:.0f} m",
                popup=f"**Cell Tower**<br>ID: {tower['cell']}<br>Radio: {tower['radio']}<br>MCC: {tower['mcc']}<br>MNC: {tower['net']}<br>LAC/TAC: {tower['area']}<br>Lat: {tower['lat']:.4f}<br>Lon: {tower['lon']:.4f}<br>Range: {tower['range']:.0f} m<br>Inoperative: {tower['inoperative']}"
            ).add_to(m)

    # Add drone marker and simulate signal reception
    # Prepare error text for display next to drone
    error_text = ""
    if st.session_state['localization_error_m'] is not None:
        error_text = f"<br><span style='font-size: 10px; color: black; font-weight: bold;'>Error: {st.session_state['localization_error_m']:.2f} m</span>"

    drone_icon = folium.DivIcon(
        html=f'<div style="font-size: {DRONE_ICON_SIZE[0]}px; text-align: center; line-height: 1;">{DRONE_EMOJI}{error_text}</div>',
        icon_size=(DRONE_ICON_SIZE[0], DRONE_ICON_SIZE[1] + 15), # Increase height slightly for text
        icon_anchor=(DRONE_ICON_SIZE[0] / 2, DRONE_ICON_SIZE[1] / 2)
    )
    folium.Marker(
        location=st.session_state['drone_lat_lon'],
        icon=drone_icon,
        tooltip="Drone Position",
        popup=f"Drone<br>Lat: {st.session_state['drone_lat_lon'][0]:.4f}<br>Lon: {st.session_state['drone_lat_lon'][1]:.4f}"
    ).add_to(m)

    # IMPORTANT: Clear these lists at the beginning of the calculation cycle
    drone_rssi_readings = []
    st.session_state['connected_towers_info'] = [] 

    current_time_for_noise = time.time() 

    if not st.session_state['cell_towers_df_filtered'].empty:
        for idx, tower in st.session_state['cell_towers_df_filtered'].iterrows():
            if tower['inoperative']:
                continue

            distance_to_tower = tower['distance_to_drone']
            
            rssi_dbm, rssi_percentage = calculate_rssi(distance_to_tower, overall_attenuation_db)
            
            if f'noise_offset_{tower["cell"]}' not in st.session_state:
                st.session_state[f'noise_offset_{tower["cell"]}'] = random.uniform(-constant_noise_offset, constant_noise_offset)
            
            time_noise = time_noise_magnitude * math.sin(current_time_for_noise * time_noise_frequency * 2 * math.pi)

            rssi_dbm_with_noise = rssi_dbm + st.session_state[f'noise_offset_{tower["cell"]}'] + time_noise
            
            min_dbm = -110
            max_dbm = -50
            rssi_percentage_with_noise = max(0, min(100, (rssi_dbm_with_noise - min_dbm) / (max_dbm - min_dbm) * 100))

            connection_threshold_dbm = -100 
            if rssi_dbm_with_noise >= connection_threshold_dbm:
                st.session_state['connected_towers_info'].append({
                    'cell': tower['cell'], 
                    'lat': tower['lat'],
                    'lon': tower['lon'],
                    'rssi_dbm': rssi_dbm_with_noise,
                    'rssi_percentage': rssi_percentage_with_noise, 
                    'radio': tower['radio'],
                    'mcc': tower['mcc'],
                    'net': tower['net'],
                    'area': tower['area'],
                    'distance_to_drone': distance_to_tower 
                })
                drone_rssi_readings.append({
                    'tower_id': tower['cell'], 
                    'rssi_dbm': rssi_dbm_with_noise
                })

        # --- Filter towers for trilateration based on RSSI threshold and limit ---
        trilateration_eligible_towers = [
            t for t in st.session_state['connected_towers_info'] 
            if t['rssi_dbm'] >= TRILATERATION_MIN_RSSI_DBM
        ]
        # Sort by RSSI (descending) and take the top N
        trilateration_eligible_towers_sorted = sorted(trilateration_eligible_towers, key=lambda x: x['rssi_dbm'], reverse=True)
        trilateration_towers_final = trilateration_eligible_towers_sorted[:MAX_TRILATERATION_TOWERS]

        # Prepare drone_rssi_readings specifically for the selected trilateration towers
        drone_rssi_readings_for_trilateration = [
            {'tower_id': t['cell'], 'rssi_dbm': t['rssi_dbm']}
            for t in trilateration_towers_final
        ]

        # Limit dotted green lines to max 5 closest towers (using all connected for this, not just trilateration ones)
        connected_towers_info_sorted_by_distance = sorted(st.session_state['connected_towers_info'], key=lambda x: x['distance_to_drone'])
        
        for tower in connected_towers_info_sorted_by_distance[:MAX_DISPLAYED_CONNECTION_LINES]:
            folium.PolyLine(
                locations=[st.session_state['drone_lat_lon'], [tower['lat'], tower['lon']]],
                color="green",
                weight=1,
                opacity=0.7,
                dash_array="5, 5"
            ).add_to(m)

    # Estimate drone position (pass the specifically filtered and prepared lists)
    st.session_state['estimated_drone_lat_lon'], trilateration_error = estimate_drone_position(
        trilateration_towers_final, drone_rssi_readings_for_trilateration, overall_attenuation_db
    )
    
    if st.session_state['estimated_drone_lat_lon']:
        # Add estimated position marker to the map object 'm'
        folium.CircleMarker(
            location=st.session_state['estimated_drone_lat_lon'],
            radius=5,
            color='red',
            fill=True,
            fill_color='red',
            fill_opacity=0.7,
            tooltip="Estimated Drone Position"
        ).add_to(m)

        # Calculate error
        actual_lat, actual_lon = st.session_state['drone_lat_lon']
        est_lat, est_lon = st.session_state['estimated_drone_lat_lon']
        st.session_state['localization_error_m'] = haversine(actual_lat, actual_lon, est_lat, est_lon)

        # Add estimated localization error circle
        if st.session_state['localization_error_m'] is not None and st.session_state['localization_error_m'] > 0:
            folium.Circle(
                location=st.session_state['estimated_drone_lat_lon'],
                radius=st.session_state['localization_error_m'], 
                color='darkyellow',
                fill=True,
                fill_color='lightyellow',
                fill_opacity=0.4, 
                tooltip=f"Estimated Error: {st.session_state['localization_error_m']:.2f} m"
            ).add_to(m)
    else:
        st.session_state['localization_error_m'] = None

    # Store the generated map in session state
    st.session_state['m_folium_obj'] = m


# --- Display Results (always render based on session state) ---
if st.session_state['m_folium_obj']:
    st.components.v1.html(
        st.session_state['m_folium_obj']._repr_html_(),
        height=620,
        scrolling=False,
    )
else:
    st.warning("Map not available. Please load cell tower data and click 'Update Calculation'.")

# --- JavaScript for click handling (this remains outside the button block) ---
st.markdown(
    """
    <script>
    function setStreamlitValue(key, value) {
        if (window.parent.streamlit) {
            window.parent.streamlit.setComponentValue(key, value);
        } else {
            console.warn("Streamlit object not found in parent window.");
        }
    }

    const mapIframe = document.querySelector('iframe[title="streamlit_component"]');

    if (mapIframe) {
        mapIframe.onload = function() {
            try {
                const map = mapIframe.contentWindow.document.querySelector('.folium-map');
                if (map && map.leafletElement) {
                    map.leafletElement.on('click', function(e) {
                        const lat = e.latlng.lat;
                        const lon = e.latlng.lon;
                        setStreamlitValue('drone_map_click', {lat: lat, lon: lon});
                    });
                } else {
                    console.warn("Leaflet map object not found inside the iframe's contentWindow.");
                }
            } catch (error) {
                console.error("Error attaching click listener to map:", error);
            }
        };
    } else {
        console.warn("Folium map iframe not found with title 'streamlit_component'.");
    }
    </script>
    """,
    unsafe_allow_html=True
)

# Retrieve the click data using st.session_state (updated by the JS)
clicked_data = st.session_state['drone_map_click']

if clicked_data['lat'] is not None and clicked_data['lon'] is not None:
    new_lat = clicked_data['lat']
    new_lon = clicked_data['lon']
    
    # Check if the clicked position is significantly different to avoid unnecessary reruns
    if haversine(st.session_state['drone_lat_lon'][0], st.session_state['drone_lat_lon'][1], new_lat, new_lon) > 0.5: # if moved by more than 0.5 meters
        st.session_state['drone_lat_lon'] = [new_lat, new_lon]
        st.session_state['last_clicked_lat'] = new_lat
        st.session_state['last_clicked_lon'] = new_lon
        
        # Reset the click data in session state to prevent immediate re-processing
        st.session_state['drone_map_click'] = {'lat': None, 'lon': None}
        st.session_state['recalculate_flag'] = True # Trigger recalculation for new drone position
        st.rerun()

# --- Display Localization Results and Connected Towers Table ---
st.markdown("---") 

if st.session_state['estimated_drone_lat_lon']:
    st.write(f"Estimated Drone Position: Lat: {st.session_state['estimated_drone_lat_lon'][0]:.4f}, Lon: {st.session_state['estimated_drone_lat_lon'][1]:.4f}")
    
    if st.session_state['localization_error_m'] is not None:
        st.write(f"Localization Error: {st.session_state['localization_error_m']:.2f} meters")

else:
    st.warning(f"Position estimation not possible: Not enough connected towers (need at least 3). Connected: {len(st.session_state['connected_towers_info'])}")

st.subheader("Connected Cell Towers")
if st.session_state['connected_towers_info']:
    display_data = []
    sorted_display_towers = sorted(st.session_state['connected_towers_info'], key=lambda x: x['rssi_dbm'], reverse=True)
    for tower in sorted_display_towers:
        display_data.append({
            'ID': tower['cell'], 
            'Radio': tower['radio'],
            'MCC': tower['mcc'],
            'MNC': tower['net'],
            'LAC/TAC': tower['area'],
            'RSSI (dBm)': f"{tower['rssi_dbm']:.2f}",
            'Signal (%)': f"{tower['rssi_percentage']:.1f}%",
            'Distance (m)': f"{tower['distance_to_drone']:.0f}", 
            'Lat': f"{tower['lat']:.4f}",
            'Lon': f"{tower['lon']:.4f}"
        })
    st.dataframe(pd.DataFrame(display_data))
else:
    st.info("No cell towers are currently within reception range or all are inoperative.")

st.markdown("---")
st.info("Click anywhere on the map to move the drone to that location.")