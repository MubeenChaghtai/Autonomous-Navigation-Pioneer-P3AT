| Noise profile | ATE odometry (m) | ATE SLAM (m) | Final error odometry (m) | Final error SLAM (m) | Map F1 odometry | Map F1 SLAM | SLAM time per scan (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| low | 0.175 | 0.061 | 0.180 | 0.041 | 0.754 | 1.000 | 6.4 |
| moderate | 1.802 | 0.109 | 2.595 | 0.032 | 0.242 | 0.902 | 7.5 |
| high | 4.524 | 0.204 | 7.378 | 0.105 | 0.169 | 0.694 | 7.7 |
| severe | 6.561 | 0.559 | 9.189 | 0.798 | 0.085 | 0.230 | 7.0 |

| Noise profile | 50 particles | 100 particles | 250 particles | 500 particles | 1000 particles |
|---|---:|---:|---:|---:|---:|
| low | 4.9 cm / 0% | 4.6 cm / 0% | 4.1 cm / 0% | 4.2 cm / 0% | 4.1 cm / 0% |
| moderate | 5.9 cm / 0% | 5.6 cm / 0% | 5.3 cm / 0% | 5.2 cm / 0% | 5.2 cm / 0% |
| high | 5.8 cm / 0% | 5.5 cm / 0% | 5.4 cm / 0% | 5.4 cm / 0% | 5.2 cm / 0% |
| severe | 6.6 cm / 0% | 6.2 cm / 0% | 6.0 cm / 0% | 6.0 cm / 0% | 5.9 cm / 0% |

| Scenario | Variant | Particles | Recovered | Median time to recover (s) |
|---|---|---:|---:|---:|
| kidnapped | augmented | 500 | 10/10 | 29.6 |
| kidnapped | standard | 500 | 0/10 | n/a |
| global | augmented | 5000 | 7/10 | 30.0 |
| global | standard | 5000 | 5/10 | 0.4 |

| Noise profile | Scenario | Success | Collisions | Median time (s) | Mean localisation error (cm) | Max localisation error (cm) | Mean replans |
|---|---|---:|---:|---:|---:|---:|---:|
| low | known map | 10/10 | 0 | 25.0 | 3.9 | 17.1 | 11.0 |
| low | unmapped obstacles | 10/10 | 0 | 33.2 | 3.6 | 27.3 | 16.1 |
| moderate | known map | 9/10 | 0 | 28.9 | 5.2 | 21.8 | 14.5 |
| moderate | unmapped obstacles | 10/10 | 0 | 44.2 | 5.5 | 24.4 | 22.5 |
| high | known map | 8/10 | 0 | 49.0 | 5.0 | 26.7 | 22.1 |
| high | unmapped obstacles | 7/10 | 0 | 68.1 | 58.4 | 948.4 | 37.3 |
| severe | known map | 7/10 | 0 | 97.8 | 6.0 | 251.6 | 44.8 |
| severe | unmapped obstacles | 2/10 | 0 | 117.7 | 114.7 | 1076.9 | 52.1 |
