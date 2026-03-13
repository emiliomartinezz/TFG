import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm


class Optimizer:
    def __init__(self, config):
        self.optimization_mode = config.get("Optimization Mode", False)
        self.path_loss_exponent = config.get("Path loss Exponent", 2.0)
        self.coverage_threshold_db = config.get("Coverage threshold (dB)", 5)
        self.grid_resolution = config.get("Grid resolution (m)", 25)
        self.carrier_frequency_ghz = config.get("Carrier frequency (GHz)", 3.5)
        self.bandwidth_mhz = config.get("Bandwidth (MHz)", 20)

        self.num_UAVs = config["Number of UAVs"]

        self.xmin = config.get("xmin")
        self.xmax = config.get("xmax")
        self.ymin = config.get("ymin")
        self.ymax = config.get("ymax")

        self.height_candidates = config.get("Height candidates")
        self.h_ue = 1.5
        self.tx_power_dbm = config.get("tx_power_dbm", 30)
        self.noise_dbm = -174 + 10 * np.log10(self.bandwidth_mhz * 1e6)
        self.xx, self.yy = self.generate_grid(
            self.xmin, self.xmax, self.ymin, self.ymax, self.grid_resolution
        )

    def generate_grid(self, xmin, xmax, ymin, ymax, delta):
        x_points = np.arange(xmin + delta / 2, xmax, delta)
        y_points = np.arange(ymin + delta / 2, ymax, delta)
        xx, yy = np.meshgrid(x_points, y_points)
        return xx, yy

    def compute_snr(self, x_uav, y_uav, h_uav):
        d = np.sqrt(
            (self.xx - x_uav) ** 2
            + (self.yy - y_uav) ** 2
            + (h_uav - self.h_ue) ** 2
        )
        d = np.maximum(d, 1.0)

        PL = (
            32.4
            + 20 * np.log10(self.carrier_frequency_ghz)
            + 10 * self.path_loss_exponent * np.log10(d)
        )

        pr_dbm = self.tx_power_dbm - PL
        snr = pr_dbm - self.noise_dbm
        return snr

    def _compute_coverage(self, snr_grid):
        covered = snr_grid > self.coverage_threshold_db
        # print(f"Cells above threshold: {np.sum(covered)} / {covered.size}")
        return np.sum(covered) / covered.size

    def _compute_objective(self, snr_grid):
        """
        Función objetivo combinada:
        1. Prioridad: maximizar cobertura (% de celdas por encima del umbral)
        2. Desempate: maximizar la SNR media de todo el grid
        
        Esto garantiza que si dos posiciones dan la misma cobertura,
        se escoge la que mejor señal ofrece globalmente (y por tanto
        estará en una zona DIFERENTE a los drones ya colocados).
        """
        coverage = self._compute_coverage(snr_grid)
        mean_snr = np.mean(snr_grid)
        # Normalizamos la SNR media para que sea un desempate (peso pequeño)
        # La cobertura va de 0 a 1, la SNR media puede ser ~50-100 dB
        # Usamos un factor pequeño para que no domine sobre la cobertura
        return coverage + mean_snr * 1e-4

    def run(self, config_path="config.json", output_path="config_optimized.json"):
        print("Running static greedy optimization...")
        print(f"Grid size: {self.xx.shape}, Total cells: {self.xx.size}")
        print(f"Coverage threshold: {self.coverage_threshold_db} dB")
        print(f"Height candidates: {self.height_candidates}")
        print(f"Number of UAVs to place: {self.num_UAVs}\n")

        snr_current = np.full_like(self.xx, -np.inf, dtype=float)
        optimal_positions = []

        x_vals = self.xx[0, :]
        y_vals = self.yy[:, 0]

        for drone_idx in range(self.num_UAVs):
            best_objective = -np.inf
            best_position = None
            best_snr_update = None

            for x in x_vals:
                for y in y_vals:
                    for h in self.height_candidates:
                        snr_candidate = self.compute_snr(x, y, h)
                        # Cada celda toma la mejor SNR entre todos los drones
                        snr_new = np.maximum(snr_current, snr_candidate)

                        objective = self._compute_objective(snr_new)

                        if objective > best_objective:
                            best_objective = objective
                            best_position = (x, y, h)
                            best_snr_update = snr_new

            optimal_positions.append(best_position)
            snr_current = best_snr_update

            coverage = self._compute_coverage(snr_current)
            mean_snr = np.mean(snr_current)
            print(
                f"Placed UAV {drone_idx} at {best_position} "
                f"→ coverage={coverage:.4f}, mean_SNR={mean_snr:.2f} dB"
            )
        self.plot_snr_heatmap(snr_current, optimal_positions)
        # --- Escribir resultado en config.json ---
        with open(config_path, "r") as f:
            config = json.load(f)

        total_time = config["Total time (s)"]
        new_uav_data = {}

        for i, (x, y, h) in enumerate(optimal_positions):
            print(h)
            new_uav_data[str(i)] = [
                [0, 1400.00, 1000.00, 0, 0],
                [50, x, y, h, 0],
            ]

        config["uav_data"] = new_uav_data
        config["Number of UAVs"] = len(optimal_positions)

        with open(output_path, "w") as f:
            json.dump(config, f, indent=4)

        print("\nConfig updated with optimized UAV positions.")
        return optimal_positions
    
    def plot_snr_heatmap(self, snr_grid, uav_positions):
        
        fig, ax = plt.subplots(figsize=(12, 8))

        # --- Límites de color centrados en el umbral ---
        snr_min = np.min(snr_grid)
        snr_max = np.max(snr_grid)
        threshold = self.coverage_threshold_db

        # TwoSlopeNorm centra el colormap en el umbral de cobertura
        # Rojo = por debajo del umbral, Azul/Verde = por encima
        if snr_min < threshold < snr_max:
            norm = TwoSlopeNorm(vmin=snr_min, vcenter=threshold, vmax=snr_max)
        else:
            norm = None

        # --- Mapa de calor ---
        im = ax.pcolormesh(
            self.xx, self.yy, snr_grid,
            cmap='RdYlGn',
            norm=norm,
            shading='nearest'
        )

        cbar = fig.colorbar(im, ax=ax, label='SNR (dB)', pad=0.02)
        # Marcar el umbral en la barra de color
        cbar.ax.axhline(y=threshold, color='black', linewidth=2, linestyle='--')
        cbar.ax.text(
            1.3, threshold, f'  Umbral\n  {threshold} dB',
            transform=cbar.ax.get_yaxis_transform(),
            va='center', fontsize=9, fontweight='bold'
        )

        # --- Contorno en el umbral de cobertura ---
        ax.contour(
            self.xx, self.yy, snr_grid,
            levels=[threshold],
            colors='black',
            linewidths=1.5,
            linestyles='dashed'
        )

        # --- Posiciones de los UAVs ---
        for i, (x, y, h) in enumerate(uav_positions):
            ax.plot(x, y, 'k^', markersize=14, markeredgewidth=2,
                    markerfacecolor='white', zorder=5)
            ax.annotate(
                f'UAV {i}\n({x:.0f}, {y:.0f})\nh={h}m',
                xy=(x, y),
                xytext=(12, 12),
                textcoords='offset points',
                fontsize=8,
                fontweight='bold',
                color='black',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                          edgecolor='gray', alpha=0.85),
                zorder=6
            )

        # --- Info de cobertura ---
        coverage = self._compute_coverage(snr_grid)
        mean_snr = np.mean(snr_grid)
        min_snr = np.min(snr_grid)

        info_text = (
            f"Cobertura: {coverage * 100:.1f}%\n"
            f"SNR media: {mean_snr:.1f} dB\n"
            f"SNR mínima: {min_snr:.1f} dB\n"
            f"UAVs: {len(uav_positions)}\n"
            f"Resolución: {self.grid_resolution}m"
        )
        ax.text(
            0.02, 0.98, info_text,
            transform=ax.transAxes,
            verticalalignment='top',
            fontsize=9,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9)
        )

        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('SNR Coverage Heatmap — Optimized UAV Placement', fontsize=14)
        ax.set_aspect('equal', adjustable='box')

        plt.tight_layout()
        plt.savefig('snr_heatmap.png', dpi=200, bbox_inches='tight')
        print("Heatmap saved as 'snr_heatmap.png'")
        plt.show()

    def plot_grid(self, xx, yy):
        plt.figure(figsize=(8, 6))
        plt.scatter(xx, yy, s=10)
        plt.title("Discretized Coverage Grid")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.gca().set_aspect("equal", adjustable="box")
        plt.grid(True)
        plt.show()