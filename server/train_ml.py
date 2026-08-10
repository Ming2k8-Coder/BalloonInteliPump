import os
import glob
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report
import joblib

# Target log directory
LOG_DIR = "logs"
MODEL_PATH = "logs/balloon_state_model.pkl"

def load_dataset():
    """Loads all CSV log files in logs/ and aggregates them for training."""
    csv_files = glob.glob(os.path.join(LOG_DIR, "*.csv"))
    if not csv_files:
        print(f"❌ No training logs found in '{LOG_DIR}/'. Please run some test sessions first!")
        return None, None
        
    print(f"📂 Found {len(csv_files)} test log files. Parsing datasets...")
    
    dfs = []
    for file in csv_files:
        try:
            df = pd.read_csv(file)
            # Ensure required columns exist
            required_cols = ["Pressure_kPa", "Slope_kPa_s", "Voltage_V", "Current_A", "Mode", "Health_Pct", "PWM"]
            if all(col in df.columns for col in required_cols):
                # Compute rolling features per test run (to prevent bleeding between runs)
                df = compute_advanced_features(df)
                dfs.append(df)
        except Exception as e:
            print(f"⚠️ Error parsing {file}: {e}")
            
    if not dfs:
        return None, None
        
    full_df = pd.concat(dfs, ignore_index=True)
    # Drop rows that have NaNs due to rolling window margins at start of files
    full_df = full_df.dropna().reset_index(drop=True)
    return full_df, csv_files

def compute_advanced_features(df):
    """
    Computes rolling DSP and material physics features on the dataset
    to empower our Random Forest classifier.
    """
    # 1. Pressure variance (rolling 30 samples = 300ms window at 100Hz)
    df["Pressure_Var_300ms"] = df["Pressure_kPa"].rolling(window=30, min_periods=1).var().fillna(0.0)
    
    # 2. Cumulative Squeeze Work (Integral of pressure over a rolling 50 samples = 500ms window)
    df["Pressure_Integral_500ms"] = df["Pressure_kPa"].rolling(window=50, min_periods=1).sum().fillna(0.0) * 0.01  # dt = 10ms
    
    # 3. Pump Power Draw
    df["Power_W"] = df["Voltage_V"] * df["Current_A"]
    
    # 4. Power-to-Pressure Efficiency Ratio
    df["Power_to_Pressure_Ratio"] = np.where(df["Pressure_kPa"] > 0.5, df["Power_W"] / df["Pressure_kPa"], 0.0)
    
    # 5. Decay Rate under static load (Creep Index)
    df["Creep_Slope"] = np.where(df["PWM"] == 0, -df["Slope_kPa_s"], 0.0)
    
    return df

def label_data(df):
    """
    Creates target labels for our 'Insane ML decide' algorithm:
    0: STABLE (Normal operation)
    1: SQUEEZE (High pressure transient spikes)
    2: YIELDING (Entering plastic deformation zone)
    3: DANGER (Impending burst / structural failure)
    """
    print("🏷️ Labeling dataset based on material physics rules...")
    
    labels = []
    for idx, row in df.iterrows():
        p = row["Pressure_kPa"]
        slope = row["Slope_kPa_s"]
        health = row["Health_Pct"]
        mode = row["Mode"]
        
        # Physics-based heuristics for training labels
        if health <= 20.0 or p >= 45.0:
            labels.append(3) # DANGER (Pop imminent)
        elif mode == "SMART" and slope <= 0.15 and p > 15.0:
            labels.append(2) # YIELDING (Latex entering plastic expansion)
        elif slope > 1.5 and mode == "MANUAL":
            labels.append(1) # SQUEEZE (Transient play spike)
        else:
            labels.append(0) # STABLE
            
    df["Target_State"] = labels
    return df

def train_model():
    df, files = load_dataset()
    if df is None:
        return
        
    df = label_data(df)
    
    # Feature Selection (excluding target variables and raw ADC values)
    features = [
        "Pressure_kPa", "Slope_kPa_s", "Voltage_V", "Current_A", "Health_Pct",
        "Pressure_Var_300ms", "Pressure_Integral_500ms", "Power_W", "Power_to_Pressure_Ratio", "Creep_Slope"
    ]
    X = df[features]
    y = df["Target_State"]
    
    # Split
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    
    print(f"⚡ Training Random Forest Classifier on {len(X_train)} samples...")
    print(f"💻 Utilizing Intel i3-10105F CPU core execution...")
    
    # Random Forest is highly optimized for CPU multicore execution
    model = RandomForestClassifier(n_estimators=100, max_depth=12, n_jobs=-1, random_state=42)
    model.fit(X_train, y_train)
    
    # Evaluation
    y_pred = model.predict(X_test)
    print("\n📊 Model Performance Report:")
    print(classification_report(y_test, y_pred, target_names=["STABLE", "SQUEEZE", "YIELDING", "DANGER"]))
    
    # Save the trained model
    joblib.dump(model, MODEL_PATH)
    print(f"💾 Model successfully exported to: {MODEL_PATH}")
    
    # Optional: Feature Importance Analysis
    importances = model.feature_importances_
    for name, importance in zip(features, importances):
        print(f"🔍 Feature '{name}': Importance {importance*100:.2f}%")

if __name__ == "__main__":
    train_model()
