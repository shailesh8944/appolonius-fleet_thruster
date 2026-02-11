import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from PIL import Image

# Set page config
st.set_page_config(
    page_title="Waypoint Tracking Performance Analysis",
    page_icon="🚢",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Add custom CSS
st.markdown("""
    <style>
    .stTabs [data-baseweb="tab-list"] {
        gap: 2px;
    }
    .stTabs [data-baseweb="tab"] {
        height: 50px;
        white-space: pre-wrap;
        border-radius: 4px 4px 0px 0px;
        padding: 10px 16px;
        background-color: #f0f2f6;
    }
    .stTabs [aria-selected="true"] {
        background-color: #4e8df5 !important;
        color: white !important;
    }
    .highlight-card {
        background-color: #f8f9fa;
        border-radius: 10px;
        padding: 20px;
        box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        margin-bottom: 20px;
    }
    </style>
""", unsafe_allow_html=True)

# Load data
@st.cache_data
def load_data():
    df = pd.read_csv("team_plots/scores.csv")
    # Clean data - remove rows with missing scores or empty team names
    df = df.dropna(subset=['Score', 'Team'])
    # Convert score to numeric
    df['Score'] = pd.to_numeric(df['Score'], errors='coerce')
    df = df.dropna(subset=['Score'])
    
    # Process data for analysis
    # Get best run for each team (lowest score with all 4 waypoints if available)
    best_runs = []
    
    for team, team_data in df.groupby('Team'):
        # First check if any run has 4 waypoints
        waypoints_4 = team_data[team_data["Waypoints Tracked"] == 4]
        
        if len(waypoints_4) > 0:
            # Get the run with lowest score among those with 4 waypoints
            best_run = waypoints_4.loc[waypoints_4['Score'].idxmin()]
        else:
            # If no run has 4 waypoints, get the best overall score
            best_run = team_data.loc[team_data['Score'].idxmin()]
        
        best_runs.append(best_run)
    
    best_df = pd.DataFrame(best_runs)
    
    # Sort by waypoints tracked (desc) and then score (asc)
    best_df = best_df.sort_values(by=['Waypoints Tracked', 'Score'], ascending=[False, True])
    
    return df, best_df

df, leaderboard_df = load_data()

# Get trajectory images
@st.cache_data
def find_trajectory_images():
    image_files = {}
    team_plots_dir = "team_plots"
    
    if os.path.exists(team_plots_dir):
        for file in os.listdir(team_plots_dir):
            if file.endswith(".png") and "trajectories" in file:
                # Extract team name and add to dictionary
                team_name = file.split("_")[0]
                if team_name.startswith("team"):
                    image_files[team_name] = os.path.join(team_plots_dir, file)
                # Special case for MAVLAB team (stored as mavtest)
                elif team_name == "mavtest":
                    image_files[team_name] = os.path.join(team_plots_dir, file)
    
    return image_files

trajectory_images = find_trajectory_images()

# Sidebar
st.sidebar.title("Navigation")
page = st.sidebar.radio("Select a page", ["Dashboard Overview", "Leaderboard", "Gain Analysis", "Team Comparison", "Trajectory Visualization", "Raw Data"])

# Add explanation in sidebar
st.sidebar.markdown("---")
st.sidebar.subheader("About this Dashboard")
st.sidebar.markdown("""
This dashboard analyzes the performance of teams in a waypoint tracking exercise with different gain values. 
Teams were ranked based on:
1. Number of waypoints tracked (max 4)
2. Score (lower is better)
""")

st.sidebar.markdown("---")
st.sidebar.subheader("Gain Parameters")
st.sidebar.markdown("""
**Kp_o**: Outer loop proportional gain  
**Ki_o**: Outer loop integral gain  
**Kp_i**: Inner loop proportional gain  
**Kd_i**: Inner loop derivative gain
""")

# Title with custom CSS
st.markdown("""
<style>
.main-title {
    color: #2c3e50;
    font-size: 36px;
    font-weight: bold;
    text-align: center;
    margin-bottom: 20px;
    text-shadow: 2px 2px 4px rgba(0,0,0,0.1);
    background: linear-gradient(to right, #3498db, #2c3e50);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
}
</style>
<h1 class="main-title">Waypoint Tracking Performance Analysis</h1>
""", unsafe_allow_html=True)

# Dashboard Overview page
if page == "Dashboard Overview":
    st.header("Overview")
    
    # Create a summary section with key statistics
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.markdown('<div class="highlight-card">', unsafe_allow_html=True)
        st.subheader("Total Teams")
        st.markdown(f"<h1 style='text-align: center;'>{leaderboard_df['Team'].nunique()}</h1>", unsafe_allow_html=True)
        st.markdown('</div>', unsafe_allow_html=True)
    
    with col2:
        st.markdown('<div class="highlight-card">', unsafe_allow_html=True)
        st.subheader("Teams with All Waypoints")
        successful_teams = leaderboard_df[leaderboard_df["Waypoints Tracked"] == 4]["Team"].nunique()
        st.markdown(f"<h1 style='text-align: center;'>{successful_teams}</h1>", unsafe_allow_html=True)
        st.markdown('</div>', unsafe_allow_html=True)
    
    with col3:
        st.markdown('<div class="highlight-card">', unsafe_allow_html=True)
        st.subheader("Best Score")
        best_score = leaderboard_df[leaderboard_df["Waypoints Tracked"] == 4]["Score"].min() if successful_teams > 0 else "N/A"
        st.markdown(f"<h1 style='text-align: center;'>{best_score}</h1>", unsafe_allow_html=True)
        st.markdown('</div>', unsafe_allow_html=True)
    
    # Display top 3 teams
    st.subheader("Top 3 Performing Teams")
    
    top3 = leaderboard_df.head(3)
    
    if not top3.empty:
        cols = st.columns(3)
        for i, (_, row) in enumerate(top3.iterrows()):
            with cols[i]:
                st.markdown('<div class="highlight-card">', unsafe_allow_html=True)
                st.markdown(f"<h3 style='text-align: center;'>#{i+1}: {row['Team']}</h3>", unsafe_allow_html=True)
                st.markdown(f"<p style='text-align: center;'>Score: {row['Score']:.2f}</p>", unsafe_allow_html=True)
                st.markdown(f"<p style='text-align: center;'>Waypoints: {int(row['Waypoints Tracked'])}/4</p>", unsafe_allow_html=True)
                
                # Display gain parameters
                st.markdown("<p style='text-align: center;'><b>Gains:</b></p>", unsafe_allow_html=True)
                st.markdown(f"<p style='text-align: center;'>Kp_o: {row['Kp_o']:.2f}, Ki_o: {row['Ki_o']:.2f}</p>", unsafe_allow_html=True)
                st.markdown(f"<p style='text-align: center;'>Kp_i: {row['Kp_i']:.2f}, Kd_i: {row['Kd_i']:.2f}</p>", unsafe_allow_html=True)
                
                # Try to display team trajectory if available
                team_id = row.get('Team name as per Bag data', '')
                
                # Handle special case for MAVLAB team
                if row['Team'] == "MAVLAB team":
                    team_id = "mavtest"
                    
                if team_id in trajectory_images:
                    try:
                        img = Image.open(trajectory_images[team_id])
                        st.image(img, caption=f"{row['Team']} Trajectory", use_column_width=True)
                    except Exception as e:
                        st.write(f"Trajectory image not available")
                st.markdown('</div>', unsafe_allow_html=True)
    
    # Distribution of waypoints tracked
    st.subheader("Distribution of Waypoints Tracked")
    
    waypoint_counts = df.groupby("Waypoints Tracked").size().reset_index(name='count')
    
    fig = px.pie(
        waypoint_counts, 
        values='count', 
        names='Waypoints Tracked',
        color_discrete_sequence=px.colors.qualitative.Vivid,
        title="Distribution of Waypoints Tracked Across All Attempts"
    )
    st.plotly_chart(fig, use_container_width=True)
    
    # Quick insights - simplified to just show correlation heatmap
    st.subheader("Key Insights")
    
    # Correlation heatmap
    corr_df = df[["Kp_o", "Ki_o", "Kp_i", "Kd_i", "Score", "Waypoints Tracked"]].corr()
    
    fig = px.imshow(
        corr_df, 
        text_auto=True, 
        aspect="auto",
        color_continuous_scale='RdBu_r',
        title="Correlation Between Parameters"
    )
    st.plotly_chart(fig, use_container_width=True)

# Leaderboard page
elif page == "Leaderboard":
    st.header("Team Leaderboard")
    
    # Add rank column
    leaderboard_df = leaderboard_df.reset_index(drop=True)
    leaderboard_df.index += 1
    
    # Display leaderboard with conditional formatting
    st.dataframe(
        leaderboard_df[['Team', 'Waypoints Tracked', 'Score', 'Kp_o', 'Ki_o', 'Kp_i', 'Kd_i']],
        column_config={
            "Team": st.column_config.TextColumn("Team"),
            "Waypoints Tracked": st.column_config.NumberColumn("Waypoints Tracked", help="Number of waypoints successfully tracked"),
            "Score": st.column_config.NumberColumn("Score", format="%.2f", help="Lower score is better"),
            "Kp_o": st.column_config.NumberColumn("Kp_o"),
            "Ki_o": st.column_config.NumberColumn("Ki_o"),
            "Kp_i": st.column_config.NumberColumn("Kp_i"),
            "Kd_i": st.column_config.NumberColumn("Kd_i"),
        },
        height=600,
        use_container_width=True,
        hide_index=False,
    )
    
    # Display top performers
    st.subheader("Top Performers (All 4 Waypoints Tracked)")
    top_teams = leaderboard_df[leaderboard_df["Waypoints Tracked"] == 4].sort_values(by="Score")
    
    if len(top_teams) > 0:
        fig = px.bar(
            top_teams,
            x='Team',
            y='Score',
            color='Score',
            color_continuous_scale='blues_r',  # Reversed blues (darker = better/lower score)
            labels={'Score': 'Performance Score (lower is better)'},
            title="Top Teams with All 4 Waypoints Tracked"
        )
        fig.update_layout(xaxis_title="Team", yaxis_title="Score (lower is better)")
        st.plotly_chart(fig, use_container_width=True)
    else:
        st.write("No teams tracked all 4 waypoints.")

elif page == "Gain Analysis":
    st.header("Gain Parameter Analysis")
    
    # Add tabs for different analysis views
    tabs = st.tabs(["Parameter vs Score", "Parameter Ranges", "3D Visualization"])
    
    with tabs[0]:
        # Filter to only show teams that tracked at least one waypoint
        valid_df = df[df["Waypoints Tracked"] > 0]
        
        # Scatter plot matrix for gain parameters vs score
        st.subheader("Relationship Between Gain Parameters and Score")
        
        # Create 2x2 grid of plots
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=[
                "Outer Loop P-Gain vs Score", 
                "Outer Loop I-Gain vs Score",
                "Inner Loop P-Gain vs Score", 
                "Inner Loop D-Gain vs Score"
            ]
        )
        
        # Add traces to each subplot
        params = ["Kp_o", "Ki_o", "Kp_i", "Kd_i"]
        colors = px.colors.qualitative.Plotly
        
        for i, param in enumerate(params):
            row = i // 2 + 1
            col = i % 2 + 1
            
            for j, wp in enumerate(sorted(valid_df["Waypoints Tracked"].unique())):
                subset = valid_df[valid_df["Waypoints Tracked"] == wp]
                
                fig.add_trace(
                    go.Scatter(
                        x=subset[param],
                        y=subset["Score"],
                        mode="markers",
                        marker=dict(color=colors[j % len(colors)]),
                        name=f"{wp} Waypoints",
                        showlegend=(i == 0),
                        hovertemplate=
                            f"<b>%{{text}}</b><br>" +
                            f"{param}: %{{x}}<br>" +
                            "Score: %{{y}}<br>" +
                            "<extra></extra>",
                        text=subset["Team"],
                    ),
                    row=row, col=col
                )
        
        # Update layout
        fig.update_layout(
            height=600,
            legend=dict(
                yanchor="top",
                y=0.99,
                xanchor="right",
                x=0.99
            )
        )
        
        fig.update_xaxes(title_text="Gain Value", row=2, col=1)
        fig.update_xaxes(title_text="Gain Value", row=2, col=2)
        fig.update_yaxes(title_text="Score", row=1, col=1)
        fig.update_yaxes(title_text="Score", row=2, col=1)
        
        st.plotly_chart(fig, use_container_width=True)
    
    with tabs[1]:
        # Find the successful gain combinations
        st.subheader("Gain Parameter Ranges for Successful Tracking")
        
        successful_df = df[df["Waypoints Tracked"] == 4]
        
        if len(successful_df) > 0:
            fig = make_subplots(rows=1, cols=4, 
                            subplot_titles=("Kp_o Range", "Ki_o Range", "Kp_i Range", "Kd_i Range"))
            
            for i, param in enumerate(["Kp_o", "Ki_o", "Kp_i", "Kd_i"]):
                fig.add_trace(
                    go.Box(
                        y=successful_df[param],
                        name=param,
                        boxmean=True,
                        marker_color='royalblue'
                    ),
                    row=1, col=i+1
                )
            
            fig.update_layout(height=400, title_text="Successful Gain Parameter Ranges (Teams Tracking All 4 Waypoints)")
            st.plotly_chart(fig, use_container_width=True)
            
            # Show the optimal parameter ranges based on successful runs
            st.subheader("Recommended Parameter Ranges")
            
            col1, col2, col3, col4 = st.columns(4)
            
            with col1:
                st.metric("Kp_o Range", 
                        f"{successful_df['Kp_o'].min():.2f} - {successful_df['Kp_o'].max():.2f}",
                        f"Median: {successful_df['Kp_o'].median():.2f}")
            
            with col2:
                st.metric("Ki_o Range", 
                        f"{successful_df['Ki_o'].min():.2f} - {successful_df['Ki_o'].max():.2f}",
                        f"Median: {successful_df['Ki_o'].median():.2f}")
            
            with col3:
                st.metric("Kp_i Range", 
                        f"{successful_df['Kp_i'].min():.2f} - {successful_df['Kp_i'].max():.2f}",
                        f"Median: {successful_df['Kp_i'].median():.2f}")
            
            with col4:
                st.metric("Kd_i Range", 
                        f"{successful_df['Kd_i'].min():.2f} - {successful_df['Kd_i'].max():.2f}",
                        f"Median: {successful_df['Kd_i'].median():.2f}")
                        
            # Table with recommended gain ranges
            st.subheader("Gain Settings of Successful Teams")
            st.dataframe(
                successful_df[['Team', 'Waypoints Tracked', 'Score', 'Kp_o', 'Ki_o', 'Kp_i', 'Kd_i']],
                use_container_width=True,
                column_config={
                    "Score": st.column_config.NumberColumn("Score", format="%.2f", help="Lower score is better"),
                }
            )
        
        else:
            st.write("No teams tracked all 4 waypoints.")
    
    with tabs[2]:
        st.subheader("3D Parameter Visualization")
        
        # Allow user to select which parameters to visualize in 3D
        col1, col2 = st.columns(2)
        
        with col1:
            param_x = st.selectbox("X-axis Parameter", ["Kp_o", "Ki_o", "Kp_i", "Kd_i"], key="param_x")
            param_y = st.selectbox("Y-axis Parameter", ["Kp_i", "Kp_o", "Ki_o", "Kd_i"], index=1, key="param_y")
        
        with col2:
            param_z = st.selectbox("Z-axis (Score/Waypoints)", ["Score", "Waypoints Tracked"], key="param_z")
            color_by = st.selectbox("Color By", ["Waypoints Tracked", "Score", "Team"], key="color_by")
        
        # Filter out extreme values for better visualization
        viz_df = df.copy()
        # Only include rows with reasonable parameters
        for param in [param_x, param_y]:
            if param in ["Kp_o", "Ki_o", "Kp_i", "Kd_i"]:
                # Get 1st and 99th percentiles to filter out extreme values
                q1, q99 = np.percentile(viz_df[param].dropna(), [1, 99])
                viz_df = viz_df[(viz_df[param] >= q1) & (viz_df[param] <= q99)]
        
        # Create 3D scatter plot
        fig = px.scatter_3d(
            viz_df,
            x=param_x,
            y=param_y,
            z=param_z,
            color=color_by,
            symbol="Waypoints Tracked" if color_by != "Waypoints Tracked" else None,
            opacity=0.7,
            title=f"3D Visualization of {param_x} vs {param_y} vs {param_z}",
            labels={
                param_x: param_x,
                param_y: param_y,
                param_z: param_z,
                color_by: color_by
            }
        )
        
        # Improve layout
        fig.update_layout(
            scene=dict(
                xaxis_title=param_x,
                yaxis_title=param_y,
                zaxis_title=param_z,
            ),
            height=700
        )
        
        st.plotly_chart(fig, use_container_width=True)
        
        st.info("""
        **How to use this 3D visualization:**
        - Select different parameters for the X and Y axes
        - Choose Score or Waypoints Tracked for the Z axis
        - Color points by team, score, or waypoints tracked
        - Rotate and zoom the plot to identify patterns and optimal parameter regions
        """)

elif page == "Team Comparison":
    st.header("Team Comparison")
    
    # Select teams to compare
    selected_teams = st.multiselect("Select teams to compare", df["Team"].unique())
    
    if selected_teams:
        filtered_df = df[df["Team"].isin(selected_teams)]
        
        # Create comparison chart - simplified to just show the best run for each team
        best_runs = []
        for team in selected_teams:
            team_data = filtered_df[filtered_df["Team"] == team]
            # Prioritize runs with all waypoints tracked
            waypoints_4 = team_data[team_data["Waypoints Tracked"] == 4]
            
            if len(waypoints_4) > 0:
                best_run = waypoints_4.loc[waypoints_4['Score'].idxmin()]
            else:
                # If no run with all waypoints, get the best overall
                best_run = team_data.loc[team_data['Score'].idxmin()]
            
            best_runs.append(best_run)
        
        best_runs_df = pd.DataFrame(best_runs)
        
        # Show comparison chart
        fig = px.bar(
            best_runs_df,
            x="Team",
            y="Score",
            color="Waypoints Tracked",
            hover_data=["Kp_o", "Ki_o", "Kp_i", "Kd_i"],
            color_continuous_scale="Viridis",
            title="Best Run Comparison"
        )
        fig.update_layout(xaxis_title="Team", yaxis_title="Score (lower is better)")
        st.plotly_chart(fig, use_container_width=True)
        
        # Display team trajectories side by side if available
        st.subheader("Team Trajectories")
        
        cols = st.columns(min(3, len(selected_teams)))
        
        for i, team in enumerate(selected_teams):
            team_data = filtered_df[filtered_df["Team"] == team]
            # Try to find the trajectory image
            team_id = team_data.iloc[0].get('Team name as per Bag data', '')
            
            # Handle special case for MAVLAB
            if team == "MAVLAB team":
                team_id = "mavtest"
            
            with cols[i % 3]:
                st.write(f"**{team}**")
                if team_id in trajectory_images:
                    try:
                        img = Image.open(trajectory_images[team_id])
                        st.image(img, caption=f"Trajectory", use_column_width=True)
                    except Exception as e:
                        st.write("Trajectory image not available")
                else:
                    st.write("No trajectory image found")
                
                # Display best score and gains
                if not team_data.empty:
                    best_run = team_data.sort_values(by=["Waypoints Tracked", "Score"], ascending=[False, True]).iloc[0]
                    st.write(f"Waypoints: {int(best_run['Waypoints Tracked'])}/4")
                    st.write(f"Score: {best_run['Score']:.2f}")
                    st.write(f"Gains: Kp_o={best_run['Kp_o']:.2f}, Ki_o={best_run['Ki_o']:.2f}, Kp_i={best_run['Kp_i']:.2f}, Kd_i={best_run['Kd_i']:.2f}")
    else:
        st.write("Please select teams to compare.")

elif page == "Trajectory Visualization":
    st.header("Trajectory Visualization")
    
    # Check if trajectory images are available
    if trajectory_images:
        # Create selectbox for team selection
        team_ids = list(trajectory_images.keys())
        selected_team_id = st.selectbox("Select Team", team_ids)
        
        if selected_team_id:
            try:
                # Handle special case for MAVLAB
                if selected_team_id == "mavtest":
                    team_name = "MAVLAB team"
                else:
                    team_name = df[df['Team name as per Bag data'] == selected_team_id]['Team'].iloc[0]
                
                st.subheader(f"{team_name} Trajectory")
                
                img = Image.open(trajectory_images[selected_team_id])
                st.image(img, caption=f"{team_name} Trajectory")
                
                # Show parameters used for this trajectory
                team_data = df[df['Team'] == team_name]
                if not team_data.empty:
                    st.subheader("Team Parameters")
                    st.dataframe(
                        team_data[['Waypoints Tracked', 'Score', 'Kp_o', 'Ki_o', 'Kp_i', 'Kd_i']],
                        use_container_width=True
                    )
                
            except Exception as e:
                st.error(f"Error loading trajectory: {e}")
        
        # Show all trajectories in a grid
        st.subheader("All Team Trajectories")
        
        # Create a grid of trajectory images
        cols = st.columns(3)
        col_idx = 0
        
        for team_id, img_path in trajectory_images.items():
            try:
                # Handle special case for MAVLAB
                if team_id == "mavtest":
                    team_name = "MAVLAB team"
                else:
                    team_name = df[df['Team name as per Bag data'] == team_id]['Team'].iloc[0]
                
                with cols[col_idx]:
                    img = Image.open(img_path)
                    st.image(img, caption=f"{team_name}", use_column_width=True)
                    
                    # Show waypoints and score
                    team_data = df[df['Team'] == team_name]
                    if not team_data.empty:
                        best_run = team_data.sort_values(by=["Waypoints Tracked", "Score"], ascending=[False, True]).iloc[0]
                        st.write(f"Waypoints: {int(best_run['Waypoints Tracked'])}/4, Score: {best_run['Score']:.2f}")
                
                col_idx = (col_idx + 1) % 3
            except Exception as e:
                pass
    else:
        st.write("No trajectory images found. Please ensure trajectory plots are available in the team_plots directory.")

elif page == "Raw Data":
    st.header("Raw Data")
    
    # Filter options
    min_waypoints = st.slider("Minimum Waypoints Tracked", 0, 4, 0)
    filtered_df = df[df["Waypoints Tracked"] >= min_waypoints]
    
    # Display raw data
    st.dataframe(filtered_df, use_container_width=True)
    
    # Download button
    csv = filtered_df.to_csv(index=False).encode('utf-8')
    st.download_button(
        "Download CSV",
        csv,
        "waypoint_tracking_data.csv",
        "text/csv",
        key='download-csv'
    )

# Footer
st.markdown("---")
st.markdown("Created for Waypoint Tracking Exercise Analysis") 