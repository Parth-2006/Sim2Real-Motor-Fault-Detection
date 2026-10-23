# Edge-AI Motor Health Monitoring System (Sim2Real Predictive Maintenance)

## 📌 Project Overview & Digital Twin
This project implements a real-time fault detection and classification system for industrial DC motors using an embedded Raspberry Pi 4. It categorizes motor states into five distinct classes: **Healthy, Bearing Fault, Winding Fault, Rotor Imbalance, and Shaft Misalignment**.

**MATLAB/Simulink Digital Twin Setup**
![Simulink Digital Twin](Screenshot%202026-09-11%20195257.png)

The architecture bridges the Simulation-to-Reality (Sim2Real) gap by first synthesizing noisy, 3-axis accelerometer data ($A_{mag}$) in a Simulink Digital Twin (shown above). This simulated data is preprocessed via a rigorous DSP pipeline to train a lightweight Random Forest machine learning classifier before physical edge deployment.

---

## ⚙️ Hardware Architecture & Circuit Integration
![Physical Hardware Setup](WhatsApp%20Image%202026-10-21%20at%2011.07.15%20PM.jpeg)

The physical edge deployment utilizes a Raspberry Pi 4 running a Python-based DSP and inference pipeline. 

*   **Edge Computer:** Raspberry Pi 4 (Linux/Python).
*   **Vibration Sensor:** ADXL345 3-Axis Accelerometer (I2C: SDA $\rightarrow$ GPIO 2, SCL $\rightarrow$ GPIO 3) mounted flush to the motor casing.
*   **Speed/Phase Sensor:** TCRT5000 IR Optical Sensor (Digital Out $\rightarrow$ GPIO 17) for $1\times$ RPM phase tracking.
*   **Actuation & Drive:** Mitsumi Brushed DC Motor driven via an L298N H-Bridge (PWM speed control).
*   **Fault Modulation Elements:** Series ceramic power resistors (1$\Omega$, 2$\Omega$, 5$\Omega$ at 5W) for Winding Faults, and off-center shaft masses for Rotor Imbalance.

---

## 🧮 Signals & Systems Theory (DSP Pipeline)
To ensure the ML model learned actual fault physics rather than static RPMs, dynamic supply voltage variation, load drift, and Band-Limited White Noise were injected into the training synthesis. 

1.  **Signal Preprocessing:** 1-second time-series windowing with 50% overlap.
2.  **Filtering:** Zero-mean detrending and 4th-order Butterworth Bandpass Filtering (5 Hz – Nyquist) to suppress DC gravity offset ($9.81 m/s^2$) and low-frequency sensor drift.
3.  **Time-Domain Features:** Mean, Variance, STD, Kurtosis, Skewness, RMS, Peak-to-Peak, Crest Factor, and Impulse Factor.
4.  **Frequency-Domain Features (PSD):** Welch’s Power Spectral Density with Hann windowing extracted Dominant Frequency, Total Spectral Power, Spectral Centroid, Spectral Bandwidth, and sub-band energy distribution.
5.  **Data Hygiene:** Zero-Mean/Unit-Variance scaling ($\mu, \sigma$) was calculated strictly on the training partition (70/15/15 split) to prevent data leakage.

![Frequency Domain PSD](Screenshot%202026-09-15%20183112.png)
![DSP Filtering Process](Screenshot%202026-09-15%20183946.png)

---

## 📊 Results & Countermeasures

**MATLAB Training Environment & Validation**
![MATLAB Training Output](Screenshot%202026-09-15%20182425_2.png)

**Random Forest Confusion Matrix**
![Random Forest Confusion Matrix](Screenshot%202026-10-21%20120653.png)

The Random Forest classifier achieved a **77.53% baseline accuracy** on realistic, noise-injected test data. 
*   **Successes:** The DSP pipeline isolated the physical signatures of Winding Faults (99.8%), Shaft Misalignment (100%), and Bearing Faults (~93%) with near-perfect accuracy.
*   **Engineering Countermeasure:** The confusion matrix revealed an expected physical overlap between a *Healthy* motor under heavy load and a *Minor Rotor Imbalance*. This was successfully mitigated in the final hardware build by integrating the TCRT5000 optical sensor for exact rotational phase synchronization.

---

## 📂 Project Structure & Reproduction
![MATLAB Directory Structure](image_ae367e.png)

**⚠️ Important Note on Dataset Sizing:**
Due to high-frequency sampling (1033 Hz), the full simulated dataset (`motor_accel_dataset_5000.mat`) exceeds 297 MB and cannot be hosted directly on GitHub. 
*   **Dataset Download:** [Insert Your Google Drive/Kaggle Link Here]
*   **Deployment Package:** The `edge_deployment_package.mat` contains the trained Random Forest model and normalized DSP parameters.
*   **Edge Script:** Run `raspi_motor_monitor.py` on the Raspberry Pi 4 to initiate real-time I2C polling and edge classification.
