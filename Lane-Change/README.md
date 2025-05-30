# Trust based observer for Cooperative Vehicle Simulation

This MATLAB project simulates cooperative lane-change and platooning scenarios with multiple vehicles, supporting various control and observer strategies, trust management, and attack resilience.

## Folder Structure

- `test/` — Main scripts and scenario files.
- `test/core/` — Core classes (Vehicle, Controller, Observer, Simulator, etc.).
- `test/core/Controller/` — Different controller implementations.
- `test/core/observer/` — Observer implementations.
- `test/core/communication/` — Communication and attack modules.
- `test/Function/` — Utility functions.
- `test/figures/`, `test/gif/`, `test/TEST/` — Output and test data.

## How to Run

1. **Open MATLAB** and set your working directory to `Lane-Change/test/`.

2. **Add Paths** (done automatically in scenario scripts, e.g., `Scenaros_direct.m`):
    ```matlab
    addpath('core/');
    addpath('core/Controller');
    addpath('core/observer');
    addpath('core/communication');
    addpath('Function');
    ```

3. **Run a Scenario**:
    - For a direct scenario:
        ```matlab
        run('Scenaros_direct.m')
        ```
    - For a lane-change scenario:
        ```matlab
        run('Scenaros_changelane.m')
        ```
    - For mean/attack analysis:
        ```matlab
        run('main_diff_attack_id_plot.m')
        ```

4. **Use the GUI** (optional):
    ```matlab
    VehicleSimulationGUI.run()
    ```

## Features

- **Controllers**: IDM, CACC, CLF-QP, CBF-CLF-QP, etc.
- **Observers**: Kalman, distributed, measurement-based.
- **Trust & Reputation**: Dynamic trust evaluation and mitigation.
- **Attack Simulation**: Inject DoS, Bogus, Faulty, Scaling, Collusion attacks.
- **Visualization**: Plot vehicle trajectories, estimation errors, trust logs.

## Customization

- **Modify scenario parameters** in the scenario scripts (number of vehicles, attack type, controller type, etc.).
- **Add new controllers/observers** by extending the respective classes in `core/Controller/` or `core/observer/`.

## Requirements

- MATLAB R2019b or newer recommended.
- No special toolboxes required for core simulation.

## Contact

For questions or contributions, please contact the project maintainer.
