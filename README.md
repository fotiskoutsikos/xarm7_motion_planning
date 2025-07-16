# Εξαμηνιαία Εργασία 1 - Ρομποτική ΙΙ

## Κινηματικός έλεγχος ρομποτικού χειριστή με πλεονάζοντες βαθμούς ελευθερίας

### Στόχος
Υλοποίηση κινηματικού ελέγχου για ρομποτικό βραχίονα xArm 7 (7 DOF) με στόχο:
1. Παρακολούθηση ευθύγραμμης τροχιάς από το τελικό στοιχείο δράσης
2. Αποφυγή δύο στατικών κυλινδρικών εμποδίων

### Βασικά Στοιχεία
- **Ρομπότ**: xArm 7 (7 βαθμοί ελευθερίας)
- **Περιβάλλον**: ROS + Gazebo
- **Τροχιά**: Ευθύγραμμη κίνηση μεταξύ σημείων P<sub>A</sub>(0.617, -0.2, 0.199) και P<sub>B</sub>(0.617, 0.2, 0.199)
- **Εμπόδια**: Δύο κύλινδροι (διάμετρος 10 cm) στις θέσεις (0.2885, -0.2, 0.400) και (0.3000, 0.2, 0.3999)

### Θεωρητική Υποστήριξη
- **Διαφορική Κινηματική**:
  ```math
  \dot{p} = J(q) \dot{q}
  ```
- **Συνάρτηση Τροχιάς** (5ου βαθμού):
  ```math
  s(t) = 10\left(\frac{t}{T}\right)^3 - 15\left(\frac{t}{T}\right)^4 + 6\left(\frac{t}{T}\right)^5
  ```
- **Εξίσωση Ελέγχου**:
  ```math
  \dot{q} = J^+ (\dot{p}_{id} + K e) + (I - J^+ J) k_0 \nabla_q V(q)
  ```

### Υλοποίηση
**Κύρια Αρχεία**:
- `kinematics.py` - Κινηματική ανάλυση και Ιακωβιανή
- `controller.py` - Αλγόριθμος ελέγχου
- `utils.py` - Βοηθητικές συναρτήσεις
- `logger.py` & `plotter.py` - Καταγραφή και οπτικοποίηση

**Παράμετροι**:
```python
T = 20 s         # Περίοδος τροχιάς
K = 0.1          # Κέρδος σφάλματος
k0 = 1           # Κέρδος αποφυγής εμποδίων
r_obstacle = 0.05 m  # Ακτίνα εμποδίων
```

### Αποτελέσματα
- **Ακρίβεια Τροχιάς**: Μέγιστη απόκλιση 1.8 mm
- **Αποφυγή Εμποδίων**: Ελάχιστη απόσταση > 5 cm
- **Ομαλότητα Κίνησης**: Χωρίς απότομες μεταβολές στις γωνιακές ταχύτητες

---

## 🔧 Παράρτημα: ROS & Εγκατάσταση

### Απαιτήσεις Συστήματος
- **Λειτουργικό**: Ubuntu 18.04 (ROS Melodic) ή 20.04 (ROS Noetic)
- **Προτεινόμενη Εγκατάσταση**: Desktop-Full

### Βήματα Εγκατάστασης
1. Εγκατάσταση ROS:
   ```bash
   # Ubuntu 18.04
   sudo apt-get install ros-melodic-desktop-full
   
   # Ubuntu 20.04
   sudo apt-get install ros-noetic-desktop-full
   ```

2. Εγκατάσταση απαιτούμενων πακέτων:
   ```bash
   sudo apt-get install ros-<distro>-moveit \
                        ros-<distro>-gazebo-ros-pkgs \
                        ros-<distro>-ros-control
   ```

3. Ρύθμιση περιβάλλοντος:
   ```bash
   echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

4. Δημιουργία workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### Εκτέλεση Προσομοιώσεων
**Για ρομποτικό βραχίονα**:
```bash
# Terminal 1: Gazebo
roslaunch xarm_gazebo xarm7_with_obstacles.launch

# Terminal 2: Έλεγχος
roslaunch robo2_redundant redundant.launch

# Terminal 3: Visualization (optional)
roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
```

---

## 📚 Συνολική Αναφορά
1. [Γραπτή Αναφορά](/documentation/%CE%93%CF%81%CE%B1%CF%80%CF%84%CE%AE%20%CE%91%CE%BD%CE%B1%CF%86%CE%BF%CF%81%CE%AC%20%CE%95%CE%BE%CE%B1%CE%BC%CE%B7%CE%BD%CE%B9%CE%B1%CE%AF%CE%B1%201.pdf) - Πλήρης ανάλυση κινηματικού ελέγχου
2. [Παράρτημα](/documentation/Appendix-robotics_II-ros_and_redundant.pdf) - Οδηγίες ROS & εγκατάστασης

**Ομάδα**:
- Φώτιος Κούτσικος (03121082)
- Άγγελος Ευστρατίου (03121113)

**Μάθημα**: Ρομποτική ΙΙ - Ευφυή Ρομποτικά Συστήματα (8ο Εξάμηνο, 2024-2025)  
**Διδάσκων**: Κ. Τζαφέστας
