# ME459 HW5: Pathfinding Comparison  

## Description  
This repository contains implementations of pathfinding algorithms and the Traveling Salesman Problem (TSP) for **ME459 Robotics and Unmanned Systems**. The focus is on solving trajectory planning and navigation problems using **Dijkstra's algorithm, A* search, and Rapidly-exploring Random Tree (RRT)**. The repository includes Python scripts for implementing these algorithms and analyzing their performance in different environments.  

## Files Included  

### **Dijkstra's Algorithm Implementation**  
- **File:** ME_459_Dijkstra.py  
- **Topics Covered:**  
  - Grid-based path planning  
  - Graph search techniques  
  - Cost estimation and shortest path computation  
  - Obstacle avoidance  

### **A* Algorithm Implementation**  
- **File:** ME_459_AStar.py  
- **Topics Covered:**  
  - Heuristic-based path planning  
  - Comparison with Dijkstra's algorithm  
  - Implementation of movement constraints  
  - TurtleBot simulation integration  

### **Additional Pathfinding Implementation**  
- **File:** ME 459 HW 5_4.py  
- **Topics Covered:**  
  - Additional A* and Dijkstra algorithm testing  
  - Performance comparisons with varying obstacle densities  
  - Execution time and total travel cost analysis  

### **Homework Problems and Documentation**  
- **File:** ME 459 HW 5.pdf  
  - Description of path planning problems and requirements  
  - Problem statements for Dijkstra's, A*, and RRT 
  - Expected output and analysis  

## Installation  
Ensure Python and the required libraries are installed before running the scripts.  

### **Required Python Packages**  
- numpy  
- matplotlib  
- math  
- itertools  

To install the necessary packages, run:  

```pip install numpy matplotlib math itertools```  

## Usage  
1. Open a terminal or Python environment.  
2. Run the desired path planning script using:  

```python ME_459_Dijkstra.py```  
```python ME_459_AStar.py```    
```python ME 459 HW 5_4.py```  

3. View generated path plots and analyze results.  

## Example Output  

### **Dijkstra's Algorithm**  
- Computes shortest path through grid-based obstacles  
- Generates x vs. y trajectory plots  

### **A* Algorithm**  
- Implements heuristic cost evaluation  
- Compares search efficiency against Dijkstra  

## Contributions  
This repository is designed for educational purposes. Feel free to modify and expand upon the existing implementations.  

## License  
This project is open for educational and research use.  

---  

**Author:** Alexander Dowell  

