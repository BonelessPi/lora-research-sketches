import numpy as np
import pandas as pd
from sklearn.mixture import GaussianMixture
import scipy.optimize
import sys

def estimate_positions(csv_path, transmitter_position, window_size=10,
                                     P_t=-30, PL_d0=40, d0=1, n=2.7,
                                     n_samples=100, n_components=2):
    """
    Estimate LoRa node positions from a unified CSV of RSSI values with use of Gaussian mixture model (GMM).

    Parameters:
    - csv_path: path to the CSV file with columns ['chipid', 'time', 'rssi']
    - transmitter_position: (x, y) known location of transmitter
    - window_size: number of recent RSSI values to use per node
    - P_t, PL_d0, d0, n: path loss model parameters
    - n_samples: number of distances to sample per node from GMM
    - n_components: number of Gaussian components in the GMM

    Returns:
    - estimated_positions: dict of chipid → estimated (x, y) position
    - gmm_models: dict of chipid → fitted Gaussian mixture model
    """

    def rssi_to_distance(rssi):
        return d0 * 10 ** ((P_t - rssi - PL_d0) / (10 * n))

    df = pd.read_csv(csv_path)
    if not {'chipid', 'time', 'rssi'}.issubset(df.columns):
        raise ValueError("CSV must contain 'chipid', 'time', 'rssi' columns")

    estimated_positions = {}
    gmm_models = {}

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
            return np.mean((pred_dist - sampled_dists)**2)

        init_guess = np.array(transmitter_position) + np.random.uniform(-10, 10, size=2)
        result = scipy.optimize.minimize(loss_fn, init_guess)
        estimated_positions[chipid] = result.x

    return estimated_positions, gmm_models

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("Usage: python pos_estimate_gmm.py <path_to_csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    transmitter_position = (0, 0) 

    positions, gmms = estimate_positions(csv_path, transmitter_position)

    for chipid, pos in positions.items():
        print(f"Estimated position for chip {chipid}: {pos}")
