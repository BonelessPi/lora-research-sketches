import numpy as np
import pandas as pd
from sklearn.mixture import GaussianMixture
import scipy.optimize
import sys

class Kalman:
    def __init__(self, process_var=1e-1, measurement_var=1.0):
        self.x = np.zeros(2)  # State: position [x, y]
        self.P = np.eye(2) * 1e3  
        self.Q = np.eye(2) * process_var  # Process noise
        self.R = np.eye(2) * measurement_var  # Measurement noise
        self.initialized = False

    def predict(self):
        # Assume stationary nodes
        return self.x, self.P

    def update(self, z):
        if not self.initialized:
            self.x = z
            self.initialized = True
            return self.x

        # Kalman gain
        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)

        # Update state
        y = z - self.x
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(2)
        self.P = (I - K) @ self.P + self.Q
        return self.x

def kalman_estimate(csv_path, transmitter_position, window_size=10,
                                        P_t=-30, PL_d0=40, d0=1, n=2.7,
                                        n_samples=100, n_components=2):
    """
    Estimate LoRa node positions using Gaussian mixture model and Kalman filtering.

    Parameters:
    - csv_path: path to combined RSSI CSV
    - transmitter_position: known (x, y) position
    - window_size: RSSI window size per chip
    - P_t, PL_d0, d0, n: path loss model parameters
    - n_samples: number of GMM samples to draw
    - n_components: number of Gaussians in the GMM

    Returns:
    - estimated_positions: dict of chipid → smoothed (x, y) position
    - gmm_models: dict of chipid → GMM model
    """

    def rssi_to_distance(rssi):
        return d0 * 10 ** ((P_t - rssi - PL_d0) / (10 * n))

    df = pd.read_csv(csv_path)
    if not {'chipid', 'time', 'rssi'}.issubset(df.columns):
        raise ValueError("CSV must contain 'chipid', 'time', 'rssi' columns")

    estimated_positions = {}
    gmm_models = {}
    kalman_filters = {}

    for chipid, group in df.groupby('chipid'):
        group = group.sort_values(by='time')
        rssi_vals = group['rssi'].dropna().values[-window_size:]

        if len(rssi_vals) < window_size:
            print(f"Skipping chip {chipid}: insufficient data")
            continue

        # Fit GMM to RSSI values
        gmm = GaussianMixture(n_components=n_components, random_state=0)
        gmm.fit(rssi_vals.reshape(-1, 1))
        gmm_models[chipid] = gmm

        # Sample RSSIs and convert to estimated distances
        sampled_rssi = gmm.sample(n_samples)[0].flatten()
        sampled_dists = np.clip(rssi_to_distance(sampled_rssi), 1e-2, 1e4)

        def loss_fn(pos):
            pred_dist = np.linalg.norm(pos - np.array(transmitter_position))
            return np.mean((pred_dist - sampled_dists) ** 2)

        init_guess = np.array(transmitter_position) + np.random.uniform(-10, 10, size=2)
        result = scipy.optimize.minimize(loss_fn, init_guess)
        raw_position = result.x

        # Apply filtering
        if chipid not in kalman_filters:
            kalman_filters[chipid] = Kalman()

        smoothed_position = kalman_filters[chipid].update(raw_position)
        estimated_positions[chipid] = smoothed_position

    return estimated_positions, gmm_models

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python kalman_position.py <path_to_csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    transmitter_position = (0, 0) 

    positions, gmms = kalman_estimate(csv_path, transmitter_position)

    for chipid, pos in positions.items():
        print(f"Estimated position for chip {chipid}: {pos}")